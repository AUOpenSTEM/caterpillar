# OpenSTEM caterpillar - http://openstem.com.au
Robot Caterpillar, 3D printed frame, Arduino controlled

Proto 1 - notes
===============
Built over December 2014, January 2015 by Arjen Lentz

Acknowledgement of heritage:
  This model is roughly based on a design from 2009
  http://letsmakerobots.com/robot/project/polymorph-caterpillar
  That one used polymorph, PixAXE controlled, with primitive sensors.

The OpenSTEM proto1 caterpillar is autonomous, running on a simple ruleset.
It moves forward until it encounters an object less than 10cm away
Then it'll back up a bit, look left and right, and see which direction has more space.
It'll decide on a direction, turn, and go back to moving forward.
The motion needs some tweaking but overall it works pretty well.

Caterpillar's eyes/ears: ultrasound module (HC-SR04) with the input and output wires combined to save pins.

The gyro/accel module (MPU-6050 based module) isn't yet used, although the base code is present.
It allows the caterpillar to figure out whether it's upright, on its left or right side, or upside down.
It can then twist and wriggle appropriately to get itself upright in case of mishaps!

LEDs can be added (antennae, tail light) - I'll do this shortly.

Control with an Arduino Pro Mini (5V) or Arduino Nano.
See the comments in the code for pinout used.
SG90 servos can be driven directly from the Arduino, as long as you power them from the battery.

I suggest using a 2S LiPo pack (7.4V) as distributing AA battery holders around the frame is a pest.
Don't use a 9V block as they hold very little charge = wasteful.
For testing you can run the whole thing via USB cable, but that causes quite a bit of drag.

The frame is 3D printed, three different brackets are used.
You can find the designs in the OpenSCAD (.scad) file.
Uncomment each type one at a time to render, export to STL and slice for printing.
Add rubber feet for traction.


WARNING!

While I'm keeping this prototype intact and alive as it's utterly cute, there will be another model.
The main issue is the hinging that attaches to the servos, the only attachment is onto the servo horn
which inevitably causes significant stress on the tiny (plastic) gears of an SG90 micro servo.

I am frankly quite astonished that people commercialise robots with a similar design.
When used normally, they'll destroy servos very quickly. So don't do that. It's uncool.


If you build one, please do let us know: photos, movies, story, we want to hear about it!
Write to us at explore@openstem.com.au

====
