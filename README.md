
# FIRST Tech challenge Practical Base Code

Check out the [Roadrunner docs](https://rr.brott.dev/docs/v1-0/tuning/).

This code incorporates, FTCDashboard, and FTCLib.
Currently this code is not going forward with Roadrunner. Instead we are using a simpler, custom drive controller.

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

NOTE: All general hardware devices(IE Cameras, huskylenses, ect) expect a particular hardware name in the object class. Listed below is all the specific names for each piece of hardware. Make sure to set the config on your robot to the EXACT name for each deivce.
>April tag Huskylens = ATHuskyLens\
>Color Detection Huskylens = COLORHuskyLens\
>Apriltag USB Camera = Webcam 1

Sensor objects are passed a hardwaremap, a telemetry, and a config name. this makes isntantiation much easier as there may be multiple of the same type of sensor per robot. This methodology may be adopted for all devices in the future.

Autonomous movement is handled with a custom drive controller, titled **Drivebot**.

This controller runs a robot-relative PID for X and Y translation, as well as a live variable heading lock at all times. This is perhaps the simplest way to achieve predictable and capable robot motion, as it does not require an understanding of path following algorithms or spline mapping.
Field relative driving based off of a coordinate system will come in the future, as well as a robust WIKI on how to use all Drivebot functions.

This code is an active work in progress as of August 2024. 
