#3dscanner#
WIP Software for a simple 3D scanner with horribly hacky software.

Hardware setup consists of two line-lasers, each at a 45 degree angle to a webcam (preferably one that opencv can adjust exposure parameters on) and an object to scan on a rotating platform driven by a stepper motor via an arduino.

Upload steppercontrol/steppercontrol.ino to an arduino and see the hardware setup section on how to set up the wiring.

The software does some actual scanning, but there are no usable results yet.

###Hardware Setup###
The steppercontrol.ino sketch should work out of the box with an arduino uno, a a4988 stepper motor control breakout board (for example the one made by Sainsmart) and a bipolar stepper motor.

Use a 12v (>1A) power supply to power the arduino so the stepper motor can get its power from the arduino's VIN pin, or power the a4988 board with 12v directly.

The sketch uses the following pins per default:

From arduino to a4988:
- Digital 2 and 3 to lasers (left and right, respectively)
- Digital 4 to Enable*
- Digital 5 to MS1*
- Digital 6 to MS2*
- Digital 7 to MS3*
- Digital 9 to Reset*
- Digital 10 to Sleep*
- Digital 11 to Step
- Digital 12 to Dir*
- 5V to VCC
- GND to Ground
- VIN to VMOT
(*: Optional)

If not using the optional pins, bridge "Sleep" and "Reset" on the a4988 board, otherwise the motor won't do anything.

- Without enable, reset and sleep, the motor will always draw current
- Without MS1-3, microstepping is disabled
- Without Dir, the stepper direction cannot be reversed

From a4988 to steppers:
- 1A and 1B to one motor coil, 2A and 2B to the other. (You can figure out which pins belong to one coil with an ohmmeter, pins connected to the same coil will have a measurable resistance) If the motor moves the wrong way either switch the direction in the sketch or connect one coil the other way around.

###Dependencies###
- OpenCV
- numpy
- pySerial
