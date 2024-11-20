# Auto Feeder Arduino Sketch

The Assistive Feeder is a robot arm with a spoon that lets those who would
need human assistance to eat a meal feed themselves more independently.

The robot scoops food out of plates or bowls, and its scooping path can be reprogrammed
to work with almost any circular plate or bowl. It can store multiple bowl/plate profiles.

**Function of each Arduino pin for this project:**
---
&emsp; 0: Serial RX (unused)\
&emsp; 1: Serial TX (unused)\
&emsp; 2: User input switch\
&emsp; 3: DC motor A power pwm (high = full power)\
&emsp; 4: debug pin (unused in final design)\
&emsp; 5: linkage 1 servo signal\
&emsp; 6: linkage 2 servo signal\
&emsp; 7: Joystick button input\
&emsp; 8: DC motor B brake (high = brake)\
&emsp; 9: DC motor A brake (high = brake)\
&emsp; 10: Warning LED output\
&emsp; 11: DC motor B power pwm (high = full power)\
&emsp; 12: Servo power "direction" pin (high = forward)\
&emsp; 13: DC motor B direction pin (high = forward)

&emsp; A0: Servo current sensing\
&emsp; A1: DC Motor current sensing\
&emsp; A2: Servo motor voltage sensing\
&emsp; A3: Joystick x axis\
&emsp; A4: Joystick y axis\
&emsp; A5: Potentiometer signal

---

Doxygen for documentation. (https://www.doxygen.nl/manual/)

---

<sub>University of Maryland - A. James Clark School of Engineering</sub>