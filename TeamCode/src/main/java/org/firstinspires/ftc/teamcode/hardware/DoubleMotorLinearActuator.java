package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DoubleMotorLinearActuator {
    public Motor motor1;
    public Motor motor2;
    PIDController armController;
    double countsPerInch;
    double setpoint;
    boolean isDebug = false;
    boolean PIDOn = false;

    /**
     * Sets up the constructor for a double motor Linear actuator.
     * Make sure to invert the motor so that Positive is up
     *NOTE: THIS ACTUATOR WILL RESET ENCODER COUNTS WHEN INITIALIZED
     * @param motor1 Motor for the arm
     * @param motor2 Motor for the arm
     * @param countsPerInch Motor counts per inch traveled. Best found emperically
     * @param invertMotor1  True/false for direction inversion
     * @param invertMotor2  True/false for direction inversion
     * @param PIDConstants Constants for arm PID tuning
     */
    public DoubleMotorLinearActuator(Motor motor1, Motor motor2, double countsPerInch, boolean invertMotor1 ,boolean invertMotor2, PIDCoefficients PIDConstants){
        this.motor1 = motor1;
        this.motor2 = motor2;

        armController = new PIDController(PIDConstants.p, PIDConstants.i, PIDConstants.d);

        motor1.setInverted(invertMotor1);
        motor2.setInverted(invertMotor2);
        motor1.resetEncoder();
        motor2.resetEncoder();

    }

    /**
     * Returns the current extension of the actuator in inches
     */
    public double getInches() {
        return (motor1.getCurrentPosition() / countsPerInch);
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
     * @param speed the actuator speed
     */
    public void setOutput(double speed) {
        PIDOn = false;
        motor1.set(speed);
        motor2.set(speed);
    }

    //zero the encoder
    public void setZero() {
        motor1.resetEncoder();
        motor2.resetEncoder();
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
    public void Periodic(){
        // Enable/disable PID
        if (PIDOn) {
            motor1.set(armController.calculate(getInches(), setpoint));
            motor2.set(armController.calculate(getInches(), setpoint));
        }  else {
          //  telemetry.addData("MOTOR PID IS OFF", 0);
        }

        //Debug periodic
        if (isDebug) {
          /*  telemetry.addData("DEBUG IS ON", "");
            telemetry.addData("INCHES", getInches());
            telemetry.addData("MOTOR COUNTS", motor.getCurrentPosition());
            telemetry.addData("MOTOR SETPOINT", armController.getSetPoint());
            telemetry.addData("PID OUTPUT", armController.calculate(getInches(), setpoint));
            telemetry.addData("POSITION ERROR", armController.getPositionError()); */
        }

    }
}
