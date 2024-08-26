package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class digitalSensor {

    private TouchSensor digitalTouch;
    private Telemetry telemetry;
    private String configName;


    /**
     * Sets up the constructor for a Rev touch sensor.
     * @param Map Hardware map object
     * @param telemetry Telemetry object
     * @param configName The name of the sensor in the robot config
     */
    public digitalSensor(HardwareMap Map, Telemetry telemetry, String configName) {
        this.telemetry = telemetry;

        digitalTouch = Map.get(TouchSensor.class, configName);

    }

    /**
     * Sensor periodic- will output state with telemetry. (Optional)
     */
    public void runDigitalSensor() {
        if (digitalTouch.isPressed()) {
            telemetry.addData(configName, "PRESSED");
        } else {
            telemetry.addData(configName, "NOT PRESSED");
        }
    }

    /**
     * returns the true/false value of the sensor.
     */
    public boolean isPressed() {
        return digitalTouch.isPressed();
    }
}
