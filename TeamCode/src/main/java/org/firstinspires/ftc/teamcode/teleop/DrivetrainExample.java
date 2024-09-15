package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "DrivetrainExample")
public class DrivetrainExample extends LinearOpMode {

    //Gamepad bindings
    private GamepadEx Driver;
    private GamepadEx Operator;

    //subsystems
    private MecanumDriveSubsystem m_Drive;

    @Override
    public void runOpMode() {
        //Run when initializing
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);

        Driver = new GamepadEx(gamepad1);

        telemetry.update();
        waitForStart();
        //Run immediately when starting

        while (opModeIsActive()) {
                //Periodic Opmode
                telemetry.addData(
                        "Periodic currently running",
                        "");

                //IMU Reset button
                if (Driver.getButton(GamepadKeys.Button.Y)) {
                   m_Drive.resetHeading();
                }

                //Drivetrain method
                m_Drive.Drive(Driver.getLeftX(), Driver.getLeftY(), Driver.getRightX(), Driver.getButton(GamepadKeys.Button.RIGHT_BUMPER));
                telemetry.addData("Heading", m_Drive.getHeading());

                telemetry.update();
            }
        }
    }

