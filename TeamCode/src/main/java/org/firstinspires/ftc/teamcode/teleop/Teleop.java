package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Teleop 1")
public class Teleop extends LinearOpMode {

    //Gamepad bindings
    private GamepadEx Driver;
    private GamepadEx Operator;

    private GamepadButton imuReset;

    //subsystems
    private MecanumDriveSubsystem m_Drive;

    @Override
    public void runOpMode() {
        //Run when initializing
        m_Drive = new MecanumDriveSubsystem(hardwareMap);

        Driver = new GamepadEx(gamepad1);
        Operator = new GamepadEx(gamepad2);

        imuReset = new GamepadButton(
                Driver, GamepadKeys.Button.Y);

        waitForStart();
        //Run immediately when starting
        while(!isStopRequested()) {
            //Periodic Opmode
            m_Drive.Drive(Driver.getLeftX(), Driver.getLeftY(), Driver.getRightX(), Driver.getButton(GamepadKeys.Button.RIGHT_BUMPER));
            //IMU Reset button
            if (Driver.getButton(GamepadKeys.Button.Y)) {
                m_Drive.resetHeading();
            }
        }
    }
}
