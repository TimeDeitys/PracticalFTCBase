package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDriveSubsystem {
    public MecanumDrive Drive;

    private Motor leftFront;
    private Motor rightFront;
    private Motor leftBack;
    private Motor rightBack;

    public RevIMU imu;

    //Creates new Mecanum Drivetrain
    public MecanumDriveSubsystem(HardwareMap Map) {

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
        Drive.driveFieldCentric(y*m, -x*m, t*m,
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
        return imu.getAbsoluteHeading();
    }

    public void resetHeading() {
        imu.reset();
    }

    public void drivePeriodic() {
        //telemetry.addData("Heading", getHeading());
        //Called once per scheduler run
        //PUT PERIODIC HERE
    }
}
