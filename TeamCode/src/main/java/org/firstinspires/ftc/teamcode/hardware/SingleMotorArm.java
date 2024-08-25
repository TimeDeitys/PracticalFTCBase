package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingleMotorArm {
    public Motor motor;
    PIDController armController;
    double countsPerMecRev;
    double setpoint;
    boolean isDebug = false;
    boolean PIDOn = false;

    /**
     * Sets up the constructor for a single motor arm.
     * Make sure to invert the motor so that CCW is positive
     *NOTE: THIS ARM WILL RESET ENCODER COUNTS WHEN INITIALIZED
     * @param motor Motor for the arm
     * @param gearRatio Mechanism/Motor rotation ratio
     * @param Invert  True/false for direction inversion
     * @param PIDConstants Constants for arm PID tuning
     */
    public SingleMotorArm(Motor motor, double gearRatio, boolean Invert, PIDCoefficients PIDConstants){
        this.motor = motor;

        armController = new PIDController(PIDConstants.p, PIDConstants.i, PIDConstants.d);

        motor.setInverted(Invert);
        motor.resetEncoder();
        countsPerMecRev = motor.getCPR() * gearRatio;

    }

    /**
     * Returns the current angle of the arm in degrees
     */
    public double getAngle() {
        return (motor.getCurrentPosition() * countsPerMecRev) / 360;
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
        motor.set(speed);
    }

    //zero the encoder
    public void setZero() {
        motor.resetEncoder();
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
            motor.set(armController.calculate(getAngle(), setpoint));
        }  else {
           // telemetry.addData("MOTOR PID IS OFF", 0);
        }

        //Debug periodic
        if (isDebug) {
         /*   telemetry.addData("DEBUG IS ON", "");
            telemetry.addData("ANGLE", getAngle());
            telemetry.addData("MOTOR COUNTS", motor.getCurrentPosition());
            telemetry.addData("MOTOR SETPOINT", armController.getSetPoint());
            telemetry.addData("PID OUTPUT", armController.calculate(getAngle(), setpoint));
            telemetry.addData("ANGLE ERROR", armController.getPositionError());  */
        }

    }
}
