package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class FlywheelShooter {
    public Motor motor;
    PIDController velController;
    double setpoint;
    boolean isDebug = false;
    boolean PIDOn = false;

    public double countsPerRev;

    /**
     * Sets up the constructor for a single motor flywheel shooter.
     * Make sure to invert the motor so that Positive is out
     * @param motor Motor for the shooter
     * @param countsPerRev Motor counts per revolution.
     * @param Invert  True/false for direction inversion
     * @param PIDConstants Constants for arm PID tuning
     */
    public FlywheelShooter(Motor motor, double countsPerRev, boolean Invert, PIDCoefficients PIDConstants){
        this.motor = motor;
        this.countsPerRev = countsPerRev;

        velController = new PIDController(PIDConstants.p, PIDConstants.i, PIDConstants.d);

        motor.setInverted(Invert);
        motor.resetEncoder();

    }

    /**
     * Returns the current RPM of the motor
     */
    public double getRPM() {
        return (motor.getCorrectedVelocity() / countsPerRev);
    }

    /**
     * Sets the flywheel to a certain RPM
     * @param speed the setpoint in RPM
     */
    public void setInches(double speed) {
        setpoint = speed;
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
        velController.setPID(PIDConstants.p, PIDConstants.i, PIDConstants.d);
    }

    /**
     * sets the flywheel into debug mode - can be undone by re-initializing robot
     */
    public void setDebug(){
        isDebug = true;
    }

    /**
     * Runs all periodic functions of the flywheel - MUST BE ADDED TO PERIODIC ROBOT CODE.
     */
    public void flywheelPeriodic(){
        // Enable/disable PID
        if (PIDOn) {
            motor.set(velController.calculate(getRPM(), setpoint));
        }  else {
          //  telemetry.addData("MOTOR PID IS OFF", 0);
        }

        //Debug periodic
        if (isDebug) {
           /* telemetry.addData("DEBUG IS ON", "");
            telemetry.addData("vel", getRPM());
            telemetry.addData("MOTOR COUNTS", motor.getCurrentPosition());
            telemetry.addData("MOTOR SETPOINT", velController.getSetPoint());
            telemetry.addData("PID OUTPUT", velController.calculate(getRPM(), setpoint));
            telemetry.addData("SPEED ERROR", velController.getPositionError()); */
        }

    }
}
