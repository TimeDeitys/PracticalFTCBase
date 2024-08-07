package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class ColorHuskylens {
    private final int READ_PERIOD = 50;

    private HuskyLens huskylens;
    private Telemetry telemetry;

    private double TagX;
    private double TagY;

    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

    public ColorHuskylens(HardwareMap Map, Telemetry telemetry) {
        this.telemetry = telemetry;
        huskylens = Map.get(HuskyLens.class, "COLORHuskyLens");

        rateLimit.expire();

        if (!huskylens.knock()) {
            telemetry.addData(">>", "Problem communicating with" + huskylens.getDeviceName());
        } else {
      //      telemetry.addData(">>", "Press Start to continue with AT");
        }

        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    }

    public void runHuskyLens() {
       HuskyLens.Block[] blocks = huskylens.blocks();
       telemetry.addData("Block Count", blocks.length);
       for(int i = 0; i < blocks.length; i++) {
           TagX = blocks[i].x;
           TagY = blocks[i].y;
           telemetry.addData("block", blocks[i].toString());
           telemetry.addData("Tag X", getTagX());
           telemetry.addData("Tag Y", getTagY());
       }
    }

    //returns a path 1, 2, or 3 depending on where the block is located
    public int GetCenterstagePath() {
        HuskyLens.Block[] blocks = huskylens.blocks();
        for(int i = 0; i < blocks.length; i++) {
            if (blocks[i].x < 100) {
                return 1;
            } else if (blocks[i].x > 100 && blocks[i].x < 200) {
                return 2;
            } else if (blocks[i].x > 200) {
                return 3;
            }
        }
        return 0;
    }

    public double getTagX() {
        return TagX;
    }

    public double getTagY() {
        return TagY;
    }
}
