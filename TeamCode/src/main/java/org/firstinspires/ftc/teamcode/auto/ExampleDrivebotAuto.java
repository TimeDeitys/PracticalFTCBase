package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorHuskylens;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class ExampleDrivebotAuto extends LinearOpMode {
    //Instantiate mechanisms
    private SuperstructureSubsystem m_Superstructure;
    private MecanumDriveSubsystem m_Drive;

    private ColorHuskylens m_COLORLens;

    private int objectPosition = 0;

    @Override
    public void runOpMode() {

        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_COLORLens = new ColorHuskylens(hardwareMap, telemetry);

        while (!isStopRequested() && !opModeIsActive()) {
            objectPosition = m_COLORLens.GetCenterstagePath();
            telemetry.addData("Position during Init", objectPosition);
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        //Object position conditional statement
        if (objectPosition == 1) {
            //AUTO PATH 1
           m_Drive.AutoDriveRC(12, 0, 5);
           //Drives forward 1 foot

        } else if (objectPosition == 2) {
            //AUTO PATH 2
            m_Drive.AutoDriveRC(0, 12, 5);
            //Drives right 1 foot

        } else {
            //AUTO PATH 3
            m_Drive.AutoDriveRC(0, -12, 5);
            //Drives left 1 Foot

        }

    }
}
