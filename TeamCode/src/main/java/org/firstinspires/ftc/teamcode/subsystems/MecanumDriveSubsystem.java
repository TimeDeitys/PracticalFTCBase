package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDriveSubsystem {
    public MecanumDrive Drive;

    private Motor leftFront;
    private Motor rightFront;
    private Motor leftBack;
    private Motor rightBack;
    private Telemetry telemetry;

    private ElapsedTime runtime = new ElapsedTime();

    private double IMUOffset;

    public RevIMU imu;

    //Creates new Mecanum Drivetrain
    public MecanumDriveSubsystem(HardwareMap Map, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = new Motor(Map, "leftFront");
        rightFront = new Motor(Map, "rightFront");
        leftBack = new Motor(Map, "leftBack");
        rightBack = new Motor(Map, "rightBack");

        Drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        imu = new RevIMU(Map, "imu");
        imu.init();

    }

    public void Drive(double x, double y, double t, boolean Dampen) {
        double m;
        if (Dampen){
            m = Constants.DriveConstants.DampenMult;
        } else {
            m = Constants.DriveConstants.DriveSpeedMult;
        }
        Drive.driveFieldCentric(-y*m, -x*m, t*m,
                getHeading() + Constants.DriveConstants.IMUOffset, Constants.DriveConstants.SquareInputs);
        drivePeriodic();
    }

    public void DriveRobotRelative(double x, double y, double t, boolean Dampen) {
        double m;
        if (Dampen){
            m = Constants.DriveConstants.DampenMult;
        } else {
            m = Constants.DriveConstants.DriveSpeedMult;
        }
        Drive.driveRobotCentric(y*m, -x*m, t*m, Constants.DriveConstants.SquareInputs);
        drivePeriodic();
    }

    public double getHeading() {
        return imu.getAbsoluteHeading() - IMUOffset;
    }

    public void resetHeading() {
        IMUOffset = imu.getAbsoluteHeading();
    }

    public int getForwardTicks(){
        //assumes Forward deadwheel is plugged into LeftFront
        return leftFront.getCurrentPosition();
    }

    public int getStrafeTicks(){
        //assumes Forward deadwheel is plugged into RightBack
        return rightBack.getCurrentPosition();
    }

    public void resetDriveEncoders() {
        leftFront.stopAndResetEncoder();
        rightBack.stopAndResetEncoder();
    }

    public void drivePeriodic() {
        telemetry.addData("Heading", getHeading());
        //Called once per scheduler run
        //PUT PERIODIC HERE
    }

    //Drivebot scheduler: a custom movement utility.

    //finds the amount of ticks to move for a given distance in inches
    public static double driveDistance(double distance) {
        double drive = (Constants.AutoConstants.COUNTS_PER_INCH);
        int outputTicks = (int) Math.floor(drive * distance);
        return outputTicks;
    }

    /**
     * Autonomously drive robot centric.
     * @param Forward forward/backward in inches (forward is positive)
     * @param Right Right/left in inches (Right is positive)
     * @param TimeoutS  Allowed time to run command
     */
    public void AutoDriveRC(double Forward, double Right, double TimeoutS) {
        double initialHeading = getHeading();

        int ForwardTarget;
        int StrafeTarget;

        if(linearOpMode.opModeIsActive()) {

            //Create PID constants
            PIDCoefficients TC = Constants.AutoConstants.TranslationPID;
            PIDCoefficients SC = Constants.AutoConstants.StrafePID;
            PIDCoefficients HC = Constants.AutoConstants.HeadingPID;

            PIDController TranslationController = new PIDController(TC.p, TC.i, TC.d);
            PIDController StrafeController = new PIDController(SC.p, SC.i, SC.d);
            PIDController HeadingController = new PIDController(HC.p, HC.i, HC.d);

            TranslationController.setTolerance(Constants.AutoConstants.PIDTolerance);
            StrafeController.setTolerance(Constants.AutoConstants.PIDTolerance);

            //set target positions
            ForwardTarget = (int) driveDistance(Forward);
            StrafeTarget = (int) driveDistance(Right);

            runtime.reset();

            StrafeController.setSetPoint(StrafeTarget);
            TranslationController.setSetPoint(ForwardTarget);

            while(linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < TimeoutS) &&
            !TranslationController.atSetPoint() && !StrafeController.atSetPoint() ) {
                //Drivebot Periodic
                //actually drives the robot.
                DriveRobotRelative(StrafeController.calculate(getStrafeTicks(), StrafeTarget), TranslationController.calculate(getForwardTicks(), ForwardTarget), HeadingController.calculate(getHeading(), initialHeading), false);
                telemetry.addData("AUTO DRIVE STATUS", "RUNNING");
                telemetry.addData("X Travelled;", getForwardTicks());
                telemetry.addData("Y Travelled;", getStrafeTicks());
                telemetry.addData("Heading;", getHeading());
            }

            //Stop all motion
            DriveRobotRelative(0,0,0, false);
            resetDriveEncoders();




        }
    }
}
