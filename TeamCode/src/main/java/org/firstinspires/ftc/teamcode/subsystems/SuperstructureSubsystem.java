package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.DoubleMotorArm;
import org.firstinspires.ftc.teamcode.hardware.DoubleMotorLinearActuator;
import org.firstinspires.ftc.teamcode.hardware.DoubleServoPincher;
import org.firstinspires.ftc.teamcode.hardware.LinearActuator;
import org.firstinspires.ftc.teamcode.hardware.SingleMotorArm;

public class SuperstructureSubsystem {

    public DoubleMotorLinearActuator Elevator;
    public DoubleMotorArm Arm;
    public LinearActuator Laterator;

    public DoubleServoPincher Pincher;

    private Motor elevatorMotor1;
    private Motor elevatorMotor2;

    private Motor lateratorMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Telemetry telemetry;

    //Creates new superstructure (arm, elevator, wrist)
    public SuperstructureSubsystem(HardwareMap Map, Telemetry telemetry){

        //Create motor objects
        this.telemetry = telemetry;

        elevatorMotor1 = new Motor(Map, "elevatorMotor1");
        elevatorMotor2 = new Motor(Map, "elevatorMotor2");
        lateratorMotor = new Motor(Map, "lateratorMotor");

        leftServo = Map.get(Servo.class, "leftServo");
        rightServo = Map.get(Servo.class, "rightServo");

        //Link motors to superstructure parts
        Elevator = new DoubleMotorLinearActuator(
                elevatorMotor1,
                elevatorMotor2,
                Constants.SuperstructureConstants.elevatorCPI,
                true,
                false,
                Constants.SuperstructureConstants.elevatorPID);

        Laterator = new LinearActuator(
                lateratorMotor,
                Constants.SuperstructureConstants.lateratorCPI,
                true,
                Constants.SuperstructureConstants.lateratorPID);

        Pincher = new DoubleServoPincher(leftServo, rightServo);
    }

    public void enableDebug() {

        Elevator.setDebug();
        Laterator.setDebug();
    }

    //Sample preset - Brings all mechanisms to 0
    public void zeroPreset() {

        Elevator.setInches(0);
        Laterator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to high drop-off
    public void indexPreset() {

        Elevator.setInches(0);
        Laterator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to high drop-off
    public void pickupPreset() {

        Elevator.setInches(0);
        Laterator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to pickup
    public void mediumPreset() {

        Elevator.setInches(0);
        Laterator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to medium
    public void highPreset() {

        Elevator.setInches(0);
        Laterator.setInches(0);
    }

    /**
     * Sets the Elevator/laterator into a manual input mode where the input can be toggled by button
     * @param input1 raw input to the elevator - should be a joystick
     * @param input2 raw input to the laterator - should be a joystick
     */
    public void ManualInput(double input1, double input2) {

        Elevator.setOutput(input1);
        Laterator.setOutput(input2);
        telemetry.addData("Elevator tick", Elevator.motor1.getCurrentPosition());
        telemetry.addData("Laterator tick", Laterator.motor.getCurrentPosition());
    }

    public void periodic() {

        Elevator.Periodic();
        Laterator.Periodic();
        telemetry.addData("Elevator Inches", Elevator.getInches());
        telemetry.addData("Laterator Inches", Laterator.getInches());
    }
}
