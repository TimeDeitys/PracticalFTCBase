package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.vision.ColorHuskylens;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;

@Config
@Autonomous(name = "RRAuto", group = "Autonomous")
public class ExampleRRAuto extends LinearOpMode {
    //Instantiate mechanisms
    private SuperstructureSubsystem m_Superstructure;

    private ColorHuskylens m_COLORLens;

    private int objectPosition = 0;

    //Roadrunner items//

    //Initial starting pose of the robot
    private Pose2d StartingPose = new Pose2d(0, 0, Math.toRadians(0));

    //3 possible initial paths for the bot to take based on conditional from camera
    Action trajectoryAction1;
    Action trajectoryAction2;
    Action trajectoryAction3;

    //Final strafe out of the way
    Action trajectoryActionStrafeOut;

    @Override
    public void runOpMode() {

        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_COLORLens = new ColorHuskylens(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, StartingPose);

        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(0)
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .strafeTo(new Vector2d(0, 10))
                .turn(Math.toRadians(180))
                .lineToY(0)
                .waitSeconds(1)
                .build();

        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(0)
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .strafeTo(new Vector2d(0, 10))
                .turn(Math.toRadians(180))
                .lineToY(0)
                .waitSeconds(1)
                .build();

        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(0)
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .strafeTo(new Vector2d(0, 10))
                .turn(Math.toRadians(180))
                .lineToY(0)
                .waitSeconds(1)
                .build();

        trajectoryActionStrafeOut = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(180))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", objectPosition);
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        //Object position conditional statement
        Action trajectoryActionChosen;
        if (objectPosition == 1) {
            //AUTO PATH 1
            trajectoryActionChosen = trajectoryAction1;
            trajectoryActionChosen = trajectoryActionStrafeOut;
        } else if (objectPosition == 2) {
            //AUTO PATH 2
            trajectoryActionChosen = trajectoryAction2;
            trajectoryActionChosen = trajectoryActionStrafeOut;
        } else {
            //AUTO PATH 3
            trajectoryActionChosen = trajectoryAction3;
            trajectoryActionChosen = trajectoryActionStrafeOut;
        }

    }
}
