import 'package:flutter/material.dart';
import 'dart:math' as math;

class Led extends StatefulWidget {
  final Color color;
  final double size;

  const Led({Key? key, required this.color, this.size = 50}) : super(key: key);

  @override
  State<Led> createState() => _LedState();
}

class _LedState extends State<Led> {
  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: Size(widget.size, widget.size),
      painter: LedPainter(widget.color),
    );
  }
}

class LedPainter extends CustomPainter {
  final Color color;

  LedPainter(this.color);

  @override
  void paint(Canvas canvas, Size size) {
    final double centerX = size.width / 2;
    final double centerY = size.height / 2;
    final double radius = math.min(centerX, centerY) * 0.8;

    // 1. Main LED Body
    final Paint ledPaint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;
    canvas.drawCircle(Offset(centerX, centerY), radius, ledPaint);

    // 2. Enhanced and Larger Glow
    final Paint glowPaint = Paint()
      ..color = color.withOpacity(0.6) // Adjust opacity as needed
      ..style = PaintingStyle.fill
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 15); // Increased blur radius
    canvas.drawCircle(Offset(centerX, centerY), radius * 1.3, glowPaint); // Larger glow circle

    final Paint innerGlowPaint = Paint()
      ..color = color.withOpacity(0.4) // Adjust opacity as needed
      ..style = PaintingStyle.fill
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 20); // Increased blur radius
    canvas.drawCircle(Offset(centerX, centerY), radius * 0.9, innerGlowPaint); // Larger glow circle

    // 3. Pronounced Highlight
    final Paint highlightPaint = Paint()
      ..color = Colors.white.withOpacity(0.2) // Adjust opacity as needed
      ..style = PaintingStyle.fill;

    final double highlightRadius = radius * 0.3; // Larger highlight
    final double highlightX = centerX - radius * 0.4;
    final double highlightY = centerY - radius * 0.4;
    canvas.drawCircle(Offset(highlightX, highlightY), highlightRadius, highlightPaint);
  }

  @override
  bool shouldRepaint(LedPainter oldDelegate) => oldDelegate.color != color;
}