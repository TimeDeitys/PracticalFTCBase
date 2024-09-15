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

    @Override
    public void runOpMode() {

        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        m_Drive.AutoDriveRC(12, 0, 5);
    }
}
