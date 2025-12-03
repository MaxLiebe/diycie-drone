import 'package:flutter/material.dart';

class DroneLEDsColorModel extends ChangeNotifier {
  List<DroneLED> leds = List.generate(5, (index) => DroneLED());

  LedColorModel() {

  }

  void updateAllLeds(Color newColor) {
    for (var led in leds) {
      led.ledColor = newColor;
    }
    notifyListeners();
  }

  void updateSingleLed(int ledId, Color newColor) {
    leds[ledId].ledColor = newColor;
    notifyListeners();
  }
}

class DroneLED {
  Color ledColor = Colors.white;
}