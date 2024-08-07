package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SuperstructureSubsystem {

    public SingleMotorArm Wrist;
    public DoubleMotorArm Arm;
    public LinearActuator Elevator;

    public DoubleServoPincher Pincher;

    private Motor leftArmMotor;
    private Motor rightArmMotor;

    private Motor wristMotor;

    private Motor elevatorMotor;

    private Servo leftServo;
    private Servo rightServo;

    private Telemetry telemetry;

    //Creates new superstructure (arm, elevator, wrist)
    public SuperstructureSubsystem(HardwareMap Map, Telemetry telemetry){
        //Create motor objects
        this.telemetry = telemetry;

        leftArmMotor = new Motor(Map, "leftArmMotor");
        rightArmMotor = new Motor(Map, "rightArmMotor");
        wristMotor = new Motor(Map, "wristMotor");
        elevatorMotor = new Motor(Map, "elevatorMotor");

        leftServo = Map.get(Servo.class, "leftServo");
        rightServo = Map.get(Servo.class, "rightServo");

        //Link motors to superstructure parts
        Wrist = new SingleMotorArm(
                wristMotor,
                Constants.SuperstructureConstants.wristGearRatio,
                false,
                Constants.SuperstructureConstants.wristPID);

        Arm = new DoubleMotorArm(
                leftArmMotor,
                rightArmMotor,
                Constants.SuperstructureConstants.armGearRatio,
                true,
                false,
                Constants.SuperstructureConstants.armPID);

        Elevator = new LinearActuator(
                elevatorMotor,
                Constants.SuperstructureConstants.elevatorCPI,
                true,
                Constants.SuperstructureConstants.elevatorPID);

        Pincher = new DoubleServoPincher(leftServo, rightServo);
    }

    public void enableDebug() {
        Wrist.setDebug();
        Arm.setDebug();
        Elevator.setDebug();
    }

    //Sample preset - Brings all mechanisms to 0
    public void zeroPreset() {
        Wrist.setAngle(0);
        Arm.setAngle(0);
        Elevator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to high dropoff
    public void highPreset() {
        Wrist.setAngle(1200);
        Arm.setAngle(2800);
        Elevator.setInches(1360);
    }

    //Sample preset - Brings all mechanisms to pickup
    public void pickupPreset() {
        Wrist.setAngle(1650);
        Arm.setAngle(0);
        Elevator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to medium
    public void mediumPreset() {
        Wrist.setAngle(1130);
        Arm.setAngle(1970);
        Elevator.setInches(1360);
    }

    /**
     * Sets the arm into a manual input mode where the imput can be toggled by button
     * @param Input raw input - should be a joystick
     * @param armToggle button to input to arm
     * @param wristToggle button to input to wrist
     * @param elevatorToggle button to input to elevator
     */
    public void ManualInput(double Input, boolean armToggle, boolean wristToggle, boolean elevatorToggle) {

        if (armToggle) { Arm.setOutput(Input); }
        else { Arm.setOutput(0); }

        if (wristToggle) { Wrist.setOutput(Input); }
        else { Wrist.setOutput(0); }

        if (elevatorToggle) { Elevator.setOutput(Input); }
        else { Elevator.setOutput(0); }
    }

    public void periodic() {
        Arm.armPeriodic();
        Wrist.armPeriodic();
        Elevator.armPeriodic();
      /*  telemetry.addData("Arm Angle", Arm.getAngle());
        telemetry.addData("Wrist Angle", Wrist.getAngle());
        telemetry.addData("Elevator Inches", Elevator.getInches()); */
    }
}
