package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class SuperstructureSubsystem {

    public SingleMotorArm Wrist;
    public DoubleMotorArm Arm;
    public LinearActuator Elevator;

    private Motor leftArmMotor;
    private Motor rightArmMotor;

    private Motor wristMotor;

    private Motor elevatorMotor;

    //Creates new superstructure (arm, elevator, wrist)
    public SuperstructureSubsystem(HardwareMap Map){
        //Create motor objects
        leftArmMotor = new Motor(Map, "leftArmMotor");
        rightArmMotor = new Motor(Map, "rightArmMotor");
        wristMotor = new Motor(Map, "wristMotor");
        elevatorMotor = new Motor(Map, "elevatorMotor");

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
                false,
                true,
                Constants.SuperstructureConstants.armPID);

        Elevator = new LinearActuator(
                elevatorMotor,
                Constants.SuperstructureConstants.elevatorCPI,
                false,
                Constants.SuperstructureConstants.elevatorPID);
    }

    //Sample preset - Brings all mechanisms to 0
    public void zeroPreset() {
        Wrist.setAngle(0);
        Arm.setAngle(0);
        Elevator.setInches(0);
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
}
