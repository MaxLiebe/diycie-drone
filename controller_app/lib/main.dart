import 'dart:convert';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

import 'package:drone_controller/models/drone_communication.dart';
import 'package:drone_controller/models/led_colors.dart';
import 'package:drone_controller/widgets/color_picker_dialog.dart';
import 'package:drone_controller/widgets/led.dart';
import 'package:drone_controller/widgets/joystick.dart';

final defaultTextTheme = TextTheme(
  bodyMedium: GoogleFonts.robotoSlab(
    fontSize: 24,
    color: Colors.white,
  ),
);

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersive);
  SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight]);
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        textTheme: defaultTextTheme,
        useMaterial3: true,
      ),
      home: MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (context) => DroneCommunicationModel()),
          ChangeNotifierProvider(create: (context) => DroneLEDsColorModel())
        ],
        child: const MyHomePage(),
      ),
    );
  }
}

class MyHomePage extends StatelessWidget {
  final double _ledSize = 25;
  final double _ledHorizontalSpacing = 10;

  const MyHomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: null,
      body: Container(
        decoration: const BoxDecoration(
          image: DecorationImage(
            image: AssetImage('assets/fur.png'), // Replace with your image path
            fit: BoxFit.cover, // Adjust fit as needed
            opacity: 0.3,
          ),
          gradient: LinearGradient(
            begin: Alignment.topCenter,
            end: Alignment.bottomCenter,
            colors: [
              Color(0xFF007DEA),
              Color(0xFF004786),
            ],
          ),
        ),
        child: Column(
          children: [
            Expanded(
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: [
                      Consumer<DroneCommunicationModel>(
                        builder: (context, drone, child) {
                          return Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(drone.isConnected ? "✅ Connected to ${drone.name}" : "🛑 Not connected to any drone"),
                              Text("🛜 Drone WiFi range:  🟩 Good "),
                              Text("🔋 Battery level: ${drone.battery}% ")
                            ],
                          );
                        }
                      ),
                      Expanded(
                        child: Consumer<DroneLEDsColorModel>(
                          builder: (context, ledsManager, child) {
                            return Row(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Column(
                                  mainAxisAlignment: MainAxisAlignment.center,
                                  children: [
                                    Row(
                                      children: [
                                        GestureDetector(
                                          child: Led(color: ledsManager.leds[0].ledColor,
                                              size: _ledSize),
                                          onTap: () {
                                            showDialog(
                                              context: context,
                                              barrierDismissible: true,
                                              builder: (BuildContext context) {
                                                return ColorPickerDialog(ledsManager, ledId: 0); // Use the custom dialog widget
                                              },
                                            );
                                          }
                                        ),
                                        Padding(
                                          padding: EdgeInsets.only(
                                              left: _ledHorizontalSpacing),
                                          child: GestureDetector(
                                              child: Led(color: ledsManager.leds[1].ledColor,
                                                  size: _ledSize),
                                              onTap: () {
                                                showDialog(
                                                  context: context,
                                                  barrierDismissible: true,
                                                  builder: (BuildContext context) {
                                                    return ColorPickerDialog(ledsManager, ledId: 1); // Use the custom dialog widget
                                                  },
                                                );
                                              }
                                          ),
                                        ),
                                      ],
                                    ),
                                    GestureDetector(
                                        child: Led(color: ledsManager.leds[2].ledColor,
                                            size: _ledSize),
                                        onTap: () {
                                          showDialog(
                                            context: context,
                                            barrierDismissible: true,
                                            builder: (BuildContext context) {
                                              return ColorPickerDialog(ledsManager, ledId: 2); // Use the custom dialog widget
                                            },
                                          );
                                        }
                                    ),
                                    Row(
                                      children: [
                                        GestureDetector(
                                            child: Led(color: ledsManager.leds[3].ledColor,
                                                size: _ledSize),
                                            onTap: () {
                                              showDialog(
                                                context: context,
                                                barrierDismissible: true,
                                                builder: (BuildContext context) {
                                                  return ColorPickerDialog(ledsManager, ledId: 3); // Use the custom dialog widget
                                                },
                                              );
                                            }
                                        ),
                                        Padding(
                                          padding: EdgeInsets.only(
                                              left: _ledHorizontalSpacing),
                                          child: GestureDetector(
                                              child: Led(color: ledsManager.leds[4].ledColor,
                                                  size: _ledSize),
                                              onTap: () {
                                                showDialog(
                                                  context: context,
                                                  barrierDismissible: true,
                                                  builder: (BuildContext context) {
                                                    return ColorPickerDialog(ledsManager, ledId: 4); // Use the custom dialog widget
                                                  },
                                                );
                                              }
                                          ),
                                        ),
                                      ],
                                    ),
                                  ],
                                ),
                                Padding(
                                  padding: const EdgeInsets.only(left: 20.0),
                                  child: SizedBox(
                                    width: 250,
                                    child: Column(
                                      mainAxisAlignment: MainAxisAlignment
                                          .center,
                                      children: [
                                        Text(
                                          "Press an LED to change its color!",
                                          style: TextStyle(
                                              fontSize: 14.0), // Set the font size here
                                        ),
                                        SizedBox(height: 20),
                                        ElevatedButton(
                                          onPressed: () {
                                            showDialog(
                                              context: context,
                                              barrierDismissible: true,
                                              builder: (BuildContext context) {
                                                return ColorPickerDialog(ledsManager); // Use the custom dialog widget
                                              },
                                            );
                                          },
                                          child: Text(
                                              'Change color of all LEDs',
                                              style: Theme
                                                  .of(context)
                                                  .textTheme
                                                  .bodyMedium
                                                  ?.copyWith(fontSize: 14.0,
                                                  // Override font size
                                                  color: Colors.black)),
                                        ),
                                      ],
                                    ),
                                  ),
                                ),
                              ],
                            );
                          }
                        )
                      ),
                    ],
                  ),
                )),
            const Divider(),
            const SizedBox(height: 10),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Container(
                  margin: const EdgeInsets.only(left: 60),
                  child: const ControlJoystick(isLeft: true),
                ),
                const Expanded(child: SizedBox()),
                Container(
                  margin: const EdgeInsets.only(right: 60),
                  child: const ControlJoystick(isLeft: false),
                ),
              ],
            ),
            const SizedBox(height: 10)
          ],
        ),
      ),
    );
  }
}

// class _MyHomePageState extends State<MyHomePage> {
//   final String _droneIp = '127.0.0.1';
//   final int _httpPort = 3000;
//   final int _udpPort = 3001;
//
//   late RawDatagramSocket _udpSocket;
//   final HttpClient _httpClient;
//
//   bool _isConnected = false;
//
//   _MyHomePageState() : _httpClient = HttpClient();
//
//   @override
//   void initState() {
//     super.initState();
//     initializeDroneConnections();
//   }
//
//   void initializeDroneConnections() async {
//     try {
//       HttpClientRequest request =
//           await _httpClient.get(_droneIp, _httpPort, '/handshake');
//       await request.close();
//       //If we've gotten this far, we know there was a server on the other end that responded,
//       //we don't really care what the response was, we're good to go!
//       print("Connected to the drone successfully");
//       _isConnected = true;
//     } catch (e) {
//       _isConnected = false;
//       //TODO handle errors
//     }
//     try {
//       _udpSocket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
//       _udpSocket.send(
//           utf8.encode('Hello, server!'), InternetAddress(_droneIp), _udpPort);
//     } catch (e) {
//       //This should never happen, but good to log it anyway
//       print('UDP: Error binding to an IP: $e');
//     }
//   }
// }
