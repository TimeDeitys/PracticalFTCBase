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

    @Override
    public void runOpMode() {

        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_COLORLens = new ColorHuskylens(hardwareMap, telemetry);

        while (!isStopRequested() && !opModeIsActive()) {
            m_COLORLens.setCenterstagePathState();
            telemetry.update();
            telemetry.addData("Position", m_COLORLens.VisionStates.name());
        }
        waitForStart();

        if (isStopRequested()) return;

        switch (m_COLORLens.VisionStates) {
            case left:
                //AUTO PATH 1
                telemetry.addData("Position", m_COLORLens.VisionStates.name());
                m_Drive.AutoDriveRC(12, 0, 5);
                //Drives forward 1 foot

                break;
            case center:
                //AUTO PATH 2
                telemetry.addData("Position", m_COLORLens.VisionStates.name());
                m_Drive.AutoDriveRC(0, 12, 5);
                //Drives right 1 foot

                break;
            case right:
                //AUTO PATH 3
                telemetry.addData("Position", m_COLORLens.VisionStates.name());
                m_Drive.SetHeading(90, 5);
                //Turns to 90 degrees

                break;
            case invalid:
                telemetry.addData("Position", m_COLORLens.VisionStates.name());
                //invalid position actions go here.
                m_Drive.AutoDriveRC(12, 0, 5);

                break;
        }

    }
}
