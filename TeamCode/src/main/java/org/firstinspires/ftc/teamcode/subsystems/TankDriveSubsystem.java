package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TankDrive;

public class TankDriveSubsystem {
    public DifferentialDrive Drive;

    private Motor leftFront;
    private Motor rightFront;
    private Motor leftBack;
    private Motor rightBack;
    private Telemetry telemetry;

    private double IMUOffset;

    private MotorGroup leftMotors;
    private MotorGroup rightMotors;

    public RevIMU imu;

    //Creates new Differential Drivetrain
    public TankDriveSubsystem(HardwareMap Map, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = new Motor(Map, "leftFront");
        rightFront = new Motor(Map, "rightFront");
        leftBack = new Motor(Map, "leftBack");
        rightBack = new Motor(Map, "rightBack");

        leftMotors = new MotorGroup(leftFront, leftBack);
        rightMotors = new MotorGroup(rightFront, rightBack);



        Drive = new DifferentialDrive(leftMotors, rightMotors);

        imu = new RevIMU(Map, "imu");
        imu.init();

    }

    public void DriveArcade(double y, double t, boolean Dampen) {
        double m;
        if (Dampen){
            m = Constants.DriveConstants.DampenMult;
        } else {
            m = Constants.DriveConstants.DriveSpeedMult;
        }
        Drive.arcadeDrive(y*m, t*m, Constants.DriveConstants.SquareInputs);
        drivePeriodic();
    }

    public void DriveTank(double left, double right, boolean Dampen) {
        double m;
        if (Dampen){
            m = Constants.DriveConstants.DampenMult;
        } else {
            m = Constants.DriveConstants.DriveSpeedMult;
        }
        Drive.tankDrive(left*m, right*m, Constants.DriveConstants.SquareInputs);
        drivePeriodic();
    }

    public double getHeading() {
        return imu.getAbsoluteHeading() - IMUOffset;
    }

    public void resetHeading() {
        IMUOffset = imu.getAbsoluteHeading();
    }

    public void drivePeriodic() {
        telemetry.addData("Heading", getHeading());
        //Called once per scheduler run
        //PUT PERIODIC HERE
    }
}
