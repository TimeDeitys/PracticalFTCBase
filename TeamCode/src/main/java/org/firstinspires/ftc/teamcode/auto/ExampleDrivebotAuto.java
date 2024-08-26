package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.vision.ColorHuskylens;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;

@Config
@Autonomous(name = "DrivebotAuto", group = "Autonomous")
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
        if ((0.9 < objectPosition) && (objectPosition < 1.1)) {
            //AUTO PATH 1
           m_Drive.AutoDriveRC(12, 0, 5);
           //Drives forward 1 foot

        } else if ((1.1 < objectPosition) && (objectPosition < 2.1)) {
            //AUTO PATH 2
            m_Drive.AutoDriveRC(0, 12, 5);
            //Drives right 1 foot

        } else {
            //AUTO PATH 3
            m_Drive.SetHeading(90, 5);
            //Turns to 90 degrees

        }

    }
}
