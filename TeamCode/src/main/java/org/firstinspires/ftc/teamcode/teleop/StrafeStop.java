package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.vision.ApriltagHuskylens;
import org.firstinspires.ftc.teamcode.hardware.vision.ColorHuskylens;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;

@Autonomous(name = "StrafeStop")
public class StrafeStop extends LinearOpMode {

    //Gamepad bindings
    private GamepadEx Driver;
    private GamepadEx Operator;

    private GamepadButton imuReset;

    //subsystems
    private MecanumDriveSubsystem m_Drive;
    private SuperstructureSubsystem m_Superstructure;

    private ApriltagHuskylens m_ATLens;
    private ColorHuskylens m_ColorLens;

    public double tagXSetpoint = 150;

    private PIDController headingController;

    @Override
    public void runOpMode() {
        //Run when initializing
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_ATLens = new ApriltagHuskylens(hardwareMap, telemetry);
        m_ColorLens = new ColorHuskylens(hardwareMap, telemetry);

        Driver = new GamepadEx(gamepad1);
        Operator = new GamepadEx(gamepad2);

        headingController = new PIDController(0.005, 0, 0);

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

                if (m_ATLens.getTagX() < 150) {
                    m_Drive.DriveRobotRelative(0.3, 0, headingController.calculate(0, m_Drive.getHeading()), false);
                } else {
                    if (m_ATLens.getTagSize() < 75 ) {
                        m_Drive.DriveRobotRelative(0, 0.2, headingController.calculate(0, m_Drive.getHeading()), false);
                    }  else {
                        m_Drive.Drive(0, 0, 0, false);
                    }
                }


                telemetry.update();
            }
        }
    }

