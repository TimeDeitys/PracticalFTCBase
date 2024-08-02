# FIRST Tech challenge Practical Base Code

Check out the [Roadrunner docs](https://rr.brott.dev/docs/v1-0/tuning/).

This code incorporates Roadrunner, MeepMeep, FTCDashboard, and FTCLib.

Everything in this code operates on a compartmentalized subsystem base, in which objects are created to represent every physical mechanism of the robot.
A subsystem can be composed of multiple different mechanisms. This leads to a clean tree map of components on the robot that looks something like this:
```
 .
  └── Robot/
      └── Superstructure/
          ├── Mechanism 1/
          │   ├── motor 1
          │   └── motor 2
          └── mechanism 2/
              ├── motor 3
              └── motor 4

```

The only parameters getting passed all the way from the Robot are the hardwareMap and Telemetry objects, which are used by each subsystem to make the hardware objects, which are then finally passed to the mechanism classes.

This project includes a sample robot code used by Team 12272 Lightning's offseason robot, which has a 2 deadwheel mecanum drivetrain, a double motor arm, a single motor arm, a servo pincher, and a linear actuator. 
It also includes a simple Apriltag PID example (WIP)

NOTE: All general hardware devices(IE Cameras, huskylenses, ect) expect a particular hardware name in the object class. Listed below is all the specific names for each piece of hardware. Make sure to set the config on your robot to the EXACT name for each deivce.
>April tag Huskylens = ATHuskyLens\
>Color Detection Huskylens = COLORHuskyLens\
>Apriltag USB Camera = Webcam 1

**More will be added in the future.**

This code is an active work in progress as of July 2024. 
