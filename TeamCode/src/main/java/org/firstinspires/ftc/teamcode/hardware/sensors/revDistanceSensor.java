package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class revDistanceSensor {

    private DistanceSensor distanceSensor;
    private Telemetry telemetry;
    private String configName;


    /**
     * Sets up the constructor for a Distance sensor.
     *
     * @param Map        Hardware map object
     * @param telemetry  Telemetry object
     * @param configName The name of the sensor in the robot config
     */
    public revDistanceSensor(HardwareMap Map, Telemetry telemetry, String configName) {
        this.telemetry = telemetry;

        distanceSensor = Map.get(DistanceSensor.class, configName);
    }

    /**
     * Sensor periodic- will output state with telemetry. (Optional)
     */
    public void runDigitalSensor() {
        telemetry.addData("deviceName", distanceSensor.getDeviceName());
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));
    }

    /**
     * returns the CM distance value of the sensor.
     */
    public double getCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * returns the Inches distance value of the sensor.
     */
    public double getInches() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
