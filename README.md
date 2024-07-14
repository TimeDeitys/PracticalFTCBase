# Team Lightning 12272 - Offseason Code

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

The only parameter getting passed all the way from the Robot is the __hardwareMap__ object, which is used by each subsystem to make the hardware objects, which are then finally passed to the mechanism classes.

This code is an active work in progress as of July 2024. 

