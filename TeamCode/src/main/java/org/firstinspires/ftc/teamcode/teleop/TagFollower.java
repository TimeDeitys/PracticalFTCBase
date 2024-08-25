package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.vision.ApriltagHuskylens;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;

@TeleOp(name = "TagFollower")
public class TagFollower extends LinearOpMode {

    //Gamepad bindings
    private GamepadEx Driver;
    private GamepadEx Operator;

    private GamepadButton imuReset;

    //subsystems
    private MecanumDriveSubsystem m_Drive;
    private SuperstructureSubsystem m_Superstructure;

    private ApriltagHuskylens m_ATLens;

    public double tagXSetpoint = 150;

    private PIDController TagController;

    @Override
    public void runOpMode() {
        //Run when initializing
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_ATLens = new ApriltagHuskylens(hardwareMap, telemetry);

        Driver = new GamepadEx(gamepad1);
        Operator = new GamepadEx(gamepad2);

        TagController = new PIDController(0.005, 0, 0);

        telemetry.update();
        waitForStart();
        //Run immediately when starting


        while (opModeIsActive()) {
                //Periodic Opmode
                m_Superstructure.periodic();
                m_ATLens.runHuskyLens();
                telemetry.addData(
                        "running",
                        "Tag Alignment");

                //Drivetrain method

                if (Driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    m_Drive.DriveRobotRelative(TagController.calculate(tagXSetpoint, m_ATLens.getTagX()), Driver.getLeftX(), Driver.getRightX(), Driver.getButton(GamepadKeys.Button.RIGHT_BUMPER));
                } else {
                    m_Drive.Drive(Driver.getLeftX(), Driver.getLeftY(), Driver.getRightX(), Driver.getButton(GamepadKeys.Button.RIGHT_BUMPER));
                }
                //IMU Reset button
                if (Driver.getButton(GamepadKeys.Button.Y)) {
                    m_Drive.resetHeading();
                }

                telemetry.update();
            }
        }
    }

