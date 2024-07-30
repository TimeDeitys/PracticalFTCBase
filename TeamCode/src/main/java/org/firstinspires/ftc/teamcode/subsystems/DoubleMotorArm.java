package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DoubleMotorArm {
    public Motor motorLeft;
    public Motor motorRight;
    PIDController armController;
    double countsPerMecRev;
    double setpoint;
    boolean isDebug = false;
    boolean PIDOn = false;

    /**
     * Sets up the constructor for a Double motor arm.
     * MAKE SURE TO INVERT ATLEAST ONE MOTOR AND THAT CCW IS POSITIVE
     * PID will run off of the left motor's encoder
     *NOTE: THIS ARM WILL RESET ENCODER COUNTS WHEN INITIALIZED
     * @param motorLeft Left arm motor
     * @param motorRight Right arm motor
     * @param gearRatio Mechanism/Motor rotation ratio
     * @param leftInvert  True/false for direction inversion
     * @param rightInvert True/false for direction inversion
     * @param PIDConstants Constants for arm PID tuning
     */
    public DoubleMotorArm(Motor motorLeft, Motor motorRight, double gearRatio,
                          boolean leftInvert, boolean rightInvert, PIDCoefficients PIDConstants){
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;

        armController = new PIDController(PIDConstants.p, PIDConstants.i, PIDConstants.d);

        motorLeft.setInverted(leftInvert);
        motorRight.setInverted(rightInvert);

        motorLeft.resetEncoder();
        countsPerMecRev = motorLeft.getCPR() * gearRatio;

    }

    /**
     * Returns the current angle of the arm in degrees
     */
    public double getAngle() {
        return (motorLeft.getCurrentPosition() * countsPerMecRev) / 360;
    }

    /**
     * Sets the arm to the specified angle
     * THIS METHOD HAS NO BUILT-IN LIMITS - BE CAREFUL
     * @param angle the arm setpoint
     */
    public void setAngle(double angle) {
        setpoint = angle;
        PIDOn = true;
    }

    /**
     * Sets the arm to a percentage output
     * @param speed the arm speed
     */
    public void setOutput(double speed) {
        PIDOn = false;
        motorLeft.set(speed);
        motorRight.set(speed);
    }

    /**
     * Manually reset the PID Constants
     * @param PIDConstants takes a PIDCoefficients object for constants
     */
    public void setPIDConstants(PIDCoefficients PIDConstants) {
        armController.setPID(PIDConstants.p, PIDConstants.i, PIDConstants.d);
    }

    /**
     * sets the arm into debug mode - can be undone by re-initializing robot
     */
    public void setDebug(){
        isDebug = true;
    }

    /**
     * Runs all periodic functions of the arm - MUST BE ADDED TO PERIODIC ROBOT CODE.
     */
    public void armPeriodic(){
        // Enable/disable PID
        if (PIDOn) {
            motorLeft.set(armController.calculate(getAngle(), setpoint));
            motorRight.set(armController.calculate(getAngle(), setpoint));
        }  else {
            telemetry.addData("MOTOR PID IS OFF", "");
        }

        //Debug periodic
        if (isDebug) {
            telemetry.addData("DEBUG IS ON", "");
            telemetry.addData("ANGLE", getAngle());
            telemetry.addData("MOTOR COUNTS", motorLeft.getCurrentPosition());
            telemetry.addData("MOTOR SETPOINT", armController.getSetPoint());
            telemetry.addData("PID OUTPUT", armController.calculate(getAngle(), setpoint));
            telemetry.addData("ANGLE ERROR", armController.getPositionError());
        }

    }
}
