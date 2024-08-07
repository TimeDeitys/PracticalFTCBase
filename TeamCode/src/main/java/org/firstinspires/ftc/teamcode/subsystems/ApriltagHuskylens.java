package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class ApriltagHuskylens {
    private final int READ_PERIOD = 20;

    private HuskyLens huskylens;
    private Telemetry telemetry;

    private double TagX;
    private double TagY;
    private double TagSize;

    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

    public ApriltagHuskylens(HardwareMap Map, Telemetry telemetry) {
        this.telemetry = telemetry;
        huskylens = Map.get(HuskyLens.class, "ATHuskyLens");

        rateLimit.expire();

        if (!huskylens.knock()) {
            telemetry.addData(">>", "Problem communicating with" + huskylens.getDeviceName());
        } else {
      //      telemetry.addData(">>", "Press Start to continue with AT");
        }

        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

    }

    public void runHuskyLens() {
       HuskyLens.Block[] blocks = huskylens.blocks();
       telemetry.addData("Block Count", blocks.length);
       for(int i = 0; i < blocks.length; i++) {
           TagX = blocks[i].x;
           TagY = blocks[i].y;
           TagSize = blocks[i].width;
           telemetry.addData("block", blocks[i].toString());
           telemetry.addData("Tag X", getTagX());
           telemetry.addData("Tag Y", getTagY());
           telemetry.addData("Tag Size", getTagSize());
       }
    }

    public double getTagX() {
        return TagX;
    }

    public double getTagY() {
        return TagY;
    }

    public double getTagSize() {
        return TagSize;
    }
}
