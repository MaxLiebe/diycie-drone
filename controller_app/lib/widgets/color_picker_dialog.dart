import 'package:drone_controller/models/led_colors.dart';
import 'package:flutter/material.dart';
import 'package:flutter_colorpicker/flutter_colorpicker.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

final colorPickerTextTheme = TextTheme(
  bodyMedium: GoogleFonts.robotoSlab(
    fontSize: 24,
    color: Colors.black,
  ),
  bodyLarge: GoogleFonts.robotoSlab(
    fontSize: 24,
    color: Colors.black,
  ),
  titleMedium: GoogleFonts.robotoSlab(
    fontSize: 24,
    color: Colors.black,
  ),
  bodySmall: GoogleFonts.robotoSlab(
    fontSize: 12,
    color: Colors.black,
  ),
  headlineSmall: GoogleFonts.robotoSlab(
    fontSize: 24,
    color: Colors.black,
  ),
);

class ColorPickerDialog extends StatefulWidget {
  final DroneLEDsColorModel ledsManager;
  final int ledId;

  const ColorPickerDialog(this.ledsManager, {super.key, this.ledId = -1});

  @override
  State<ColorPickerDialog> createState() => _ColorPickerDialogState();
}

class _ColorPickerDialogState extends State<ColorPickerDialog> {
  Color currentPickedColor = Colors.red;

  void changeColor(Color color) {
    currentPickedColor = color;
  }

  @override
  Widget build(BuildContext context) {
    return Theme(
      data: Theme.of(context).copyWith(textTheme: colorPickerTextTheme),
      child: AlertDialog(
        title: const Text('Pick a color!'),
        backgroundColor: Colors.white,
        content: Column(
          children: [ColorPicker(
            hexInputBar: true,
            pickerColor: currentPickedColor,
            onColorChanged: changeColor,
            enableAlpha: false,
            displayThumbColor: true,
            pickerAreaHeightPercent: 0.5,
          ), ElevatedButton(
            child: const Text('Got it'),
            onPressed: () {
              if(widget.ledId == -1) {
                widget.ledsManager.updateAllLeds(currentPickedColor);
              }else{
                widget.ledsManager.updateSingleLed(widget.ledId, currentPickedColor);
              }
              Navigator.of(context).pop();
            },
          )],
        )
      ),
    );
  }
}
