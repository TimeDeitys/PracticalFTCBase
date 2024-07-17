package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Constants {
    public static class DriveConstants {

        public static final boolean SquareInputs = true;
        public static final double DampenMult = 0.3;
        public static final double DriveSpeedMult = 1;

        //Initial IMU angle offset for field centric in degrees
        public static final double IMUOffset = -90;
    }

    public static class SuperstructureConstants {
        public static final double wristGearRatio = 1;
        public static final PIDCoefficients wristPID = new PIDCoefficients(0, 0, 0);

        public static final double armGearRatio = 1;
        public static final PIDCoefficients armPID = new PIDCoefficients(0, 0, 0);

        public static final double elevatorCPI = 1;
        public static final PIDCoefficients elevatorPID = new PIDCoefficients(0, 0, 0);
    }
}
