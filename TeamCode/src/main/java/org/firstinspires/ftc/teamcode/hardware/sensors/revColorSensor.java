package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class revColorSensor {

   private ColorSensor colorSensor;
    private Telemetry telemetry;
    private String configName;

    private int initialAlpha;
    private int initialRed;
    private int initialBlue;

    private int sensorTolerance;


    /**
     * Sets up the constructor for a Distance sensor.
     * NOTE: The sensor will take an initial reading at this point. this will be the reference point for color.
     *
     * @param Map        Hardware map object
     * @param telemetry  Telemetry object
     * @param configName The name of the sensor in the robot config
     * @param configName sensor tolerance. needs to be tuned but should be around 20-50.
     */
    public revColorSensor(HardwareMap Map, Telemetry telemetry, String configName, int sensorTolerance) {
        this.telemetry = telemetry;
        this.sensorTolerance = sensorTolerance;

        colorSensor = Map.get(ColorSensor.class, configName);

        initialAlpha = colorSensor.alpha();
        initialRed = colorSensor.red();
        initialBlue = colorSensor.blue();
    }

    /**
     * returns the Light value differential of the sensor from initial reading.
     */
    public int getAlphaDelta() {
       return colorSensor.alpha() - initialAlpha;
    }

    /**
     * returns the Red value differential of the sensor from initial reading.
     */
    public int getRedDelta() {
        return colorSensor.red() - initialRed;
    }

    /**
     * returns the Blue value differential of the sensor from initial reading.
     */
    public int getBlueDelta() {
        return colorSensor.blue() - initialBlue;
    }

    /**
     * recalibrates the color sensor based on current reading
     */
    public void recalibrate() {
        initialAlpha = colorSensor.alpha();
        initialRed = colorSensor.red();
        initialBlue = colorSensor.blue();
    }

    /**
     * resets the sensor tolerance.
     */
    public void setTolerance(int tolerance) {
        sensorTolerance = tolerance;
    }

    /**
     * Checks for red.
     */
    public boolean isRed() {
       if (getRedDelta() > sensorTolerance) {
           return true;
       } else {
           return false;
       }
    }

    /**
     * Checks for blue.
     */
    public boolean isBlue() {
        if (getBlueDelta() > sensorTolerance) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Checks for light.
     */
    public boolean isBright() {
        if (getAlphaDelta() > sensorTolerance) {
            return true;
        } else {
            return false;
        }
    }
}
