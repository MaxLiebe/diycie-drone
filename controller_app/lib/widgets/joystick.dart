import 'package:drone_controller/models/drone_communication.dart';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';

class ControlJoystick extends StatelessWidget {
  final bool isLeft;

  const ControlJoystick({super.key, required this.isLeft});

  @override
  Widget build(BuildContext context) {
    return Container(
        decoration: BoxDecoration(
          color: Colors.white,
          shape: BoxShape.circle,
          border: Border.all(
            color: Colors.black, // Border color
            width: 3.0, // Border width
          ),
        ),
        child:
            Consumer<DroneCommunicationModel>(builder: (context, drone, child) {
          return Joystick(
            base: JoystickBase(
              size: 220,
              decoration: JoystickBaseDecoration(
                  boxShadowColor: Colors.transparent,
                  color: Colors.white,
                  drawArrows: false,
                  drawOuterCircle: false,
                  drawMiddleCircle: false,
                  drawInnerCircle: false),
              arrowsDecoration: null,
              mode: JoystickMode.all,
            ),
            stick: const ControlJoystickStick(size: 50),
            mode: JoystickMode.all,
            listener: (details) {
              drone.updateStickPosition(isLeft, details.x, details.y);
            },
          );
        }));
  }
}

class ControlJoystickStick extends JoystickStick {
  const ControlJoystickStick({super.key, super.size, super.decoration});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: EdgeInsets.all(4),
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        border: Border.all(
          color: Colors.black, // Border color
          width: 4.0, // Border width
        ),
      ),
      child: Container(
        width: size,
        height: size,
        decoration: const BoxDecoration(
          shape: BoxShape.circle,
          boxShadow: [
            BoxShadow(
              color: Colors.black,
              spreadRadius: 7,
              blurRadius: 9,
            )
          ],
          color: Colors.black,
        ),
      ),
    );
  }
}
