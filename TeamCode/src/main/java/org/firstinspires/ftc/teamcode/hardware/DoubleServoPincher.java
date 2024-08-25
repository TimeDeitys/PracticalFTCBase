package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DoubleServoPincher {
    public Servo leftServo;
    public Servo rightServo;

    public DoubleServoPincher(Servo leftServo, Servo rightServo){
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void setServos(double leftValue, double rightValue) {
        leftServo.setPosition(leftValue);
        rightServo.setPosition(rightValue);
    }

    public double getLeftServoPosition(){
        return leftServo.getPosition();
    }

    public double getRightServoPosition(){
        return rightServo.getPosition();
    }

    public void open(){
        setServos(0.5,0.5);
    }

    public void close(){
        setServos(0.8,0.2);
    }
}
