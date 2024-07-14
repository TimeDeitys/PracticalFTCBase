package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class LinearActuator {
    public Motor motor;
    PIDController armController;
    double countsPerInch;
    double setpoint;
    boolean isDebug = false;
    boolean PIDOn = false;

    /**
     * Sets up the constructor for a single motor Linear actuator.
     *NOTE: THIS ACTUATOR WILL RESET ENCODER COUNTS WHEN INITIALIZED
     * @param motor Motor for the armp
     * @param countsPerInch Motor counts per inch traveled. Best found emperically
     * @param Invert  True/false for direction inversion
     * @param PIDConstants Constants for arm PID tuning
     */
    public LinearActuator(Motor motor, double countsPerInch, boolean Invert, PIDCoefficients PIDConstants){
        this.motor = motor;

        armController = new PIDController(PIDConstants.p, PIDConstants.i, PIDConstants.d);

        motor.setInverted(Invert);
        motor.resetEncoder();

    }

    /**
     * Returns the current extension of the actuator in inches
     */
    public double getInches() {
        return (motor.getCurrentPosition() * countsPerInch);
    }

    /**
     * Sets the Actuator to a specific distance
     * THIS METHOD HAS NO BUILT-IN LIMITS - BE CAREFUL
     * @param inches the actuator setpoint
     */
    public void setInches(double inches) {
        setpoint = inches;
        PIDOn = true;
    }

    /**
     * Sets the arm to a percentage output
     * @param speed the actuuator speed
     */
    public void setOutput(double speed) {
        PIDOn = false;
        motor.set(speed);
    }

    /**
     * Manually reset the PID Constants
     * @param PIDConstants takes a PIDCoefficients object for constants
     */
    public void setPIDConstants(PIDCoefficients PIDConstants) {
        armController.setPID(PIDConstants.p, PIDConstants.i, PIDConstants.d);
    }

    /**
     * sets the actuator into debug mode - can be undone by re-initializing robot
     */
    public void setDebug(){
        isDebug = true;
    }

    /**
     * Runs all periodic functions of the actuator - MUST BE ADDED TO PERIODIC ROBOT CODE.
     */
    public void armPeriodic(){
        // Enable/disable PID
        if (PIDOn) {
            motor.set(armController.calculate(getInches(), setpoint));
        }  else {
            telemetry.addData("MOTOR PID IS OFF", "");
        }

        //Debug periodic
        if (isDebug) {
            telemetry.addData("DEBUG IS ON", "");
            telemetry.addData("INCHES", getInches());
            telemetry.addData("MOTOR COUNTS", motor.getCurrentPosition());
            telemetry.addData("MOTOR SETPOINT", armController.getSetPoint());
            telemetry.addData("PID OUTPUT", armController.calculate(getInches(), setpoint));
            telemetry.addData("POSITION ERROR", armController.getPositionError());
        }

    }
}
