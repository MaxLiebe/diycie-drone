import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:http/http.dart' as http;
import 'package:flutter/material.dart';

class DroneCommunicationModel extends ChangeNotifier {
  final String _droneIp = '192.168.4.1';

  final int _udpPort = 3001;
  late RawDatagramSocket _udpSocket;
  late final Timer _udpTimer;
  late final Timer _statusTimer;
  final int _tickRate = 60; // 60 ticks per second
  final int _statusInterval = 5000;

  final int _httpPort = 3000;
  bool _connected = false;
  String _name = 'Unknown';
  int _battery = 69;

  bool get isConnected => _connected;

  String get name => _name;
  int get battery => _battery;

  int _leftStickX = 0;
  int _leftStickY = 0;
  int _rightStickX = 0;
  int _rightStickY = 0;

  DroneCommunicationModel() {
    getUdpSocket();
    _udpTimer = Timer.periodic(Duration(milliseconds: (1000 / _tickRate).round()),
        _sendControllerInformation);
    _statusTimer = Timer.periodic(Duration(milliseconds: _statusInterval),
        _getDroneStatus);
  }

  void _getDroneStatus(t) async {
    var url = Uri.http('$_droneIp:$_httpPort', '/status');
    try {
      var response = await http.get(url).timeout(Duration(seconds: 4));
      if (response.statusCode == 200) {
        var jsonResponse = jsonDecode(response.body) as Map<String, dynamic>;
        _name = jsonResponse['name'];
        _battery = jsonResponse['battery'];
        _connected = true;
        getUdpSocket();
        notifyListeners();
      } else {
        print('Request failed with status: ${response.statusCode}.');
        _connected = false;
        notifyListeners();
        //TODO handle errors
      }
    } on TimeoutException {
      _connected = false;
      print('Failed to connect to drone. Retrying...');
      notifyListeners();
    }
  }

  void getUdpSocket() async {
    try {
      _udpSocket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    } catch (e) {
      //This should never happen, but good to log it anyway
      print('UDP: Error binding to an IP: $e');
    }
  }

  void _sendControllerInformation(Timer t) {
    if(!_connected) return;
    try {
      _udpSocket.send(
          utf8.encode(
              'X${formattedPosition(_leftStickX)}Y${formattedPosition(_leftStickY)}S${formattedPosition(_rightStickX)}L${formattedPosition(_rightStickY)}'),
          InternetAddress(_droneIp),
          _udpPort);
    } catch (e) {
      // Handle send errors
      print('Error sending UDP message: $e');
    }
  }

  void updateStickPosition(bool isLeft, double x, double y) {
    if (isLeft) {
      _leftStickX = mappedPosition(x);
      _leftStickY = mappedPosition(y);
    } else {
      _rightStickX = mappedPosition(x);
      _rightStickY = mappedPosition(y);
    }
  }

  @override
  void dispose() {
    super.dispose();
    _udpTimer.cancel();
    _udpSocket.close();
    _statusTimer.cancel();
  }

  static int mappedPosition(double pos) {
    return (pos * 127.5).round();
  }

  static String formattedPosition(int pos) {
    String sign = pos >= 0 ? "+" : "-";
    String paddedValue = pos.abs().toString().padLeft(3, '0');
    return "$sign$paddedValue";
  }
}
