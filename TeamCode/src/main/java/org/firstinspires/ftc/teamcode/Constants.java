package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Constants {
    public static class DriveConstants {

        public static final boolean SquareInputs = false;
        public static final double DampenMult = 0.3;
        public static final double DriveSpeedMult = 1;

        //Initial IMU angle offset for field centric in degrees
        public static final double IMUOffset = -90;
    }

    public static class SuperstructureConstants {
        public static final double wristGearRatio = 1;
        public static final PIDCoefficients wristPID = new PIDCoefficients(0.003, 0, 0);

        public static final double armGearRatio = 1;
        public static final PIDCoefficients armPID = new PIDCoefficients(0.005, 0, 0);

        public static final double elevatorCPI = 1;
        public static final PIDCoefficients elevatorPID = new PIDCoefficients(0.005, 0.0001, 0);
    }

    public static class AutoConstants {
        public static final double COUNTS_PER_INCH = 338.569; //Found empirically with drive tuning
        public static final PIDCoefficients TranslationPID = new PIDCoefficients(0.00035, 0, 0.000012);
        public static final PIDCoefficients StrafePID = new PIDCoefficients(0.0003, 0, 0.0);
        public static final PIDCoefficients HeadingPID = new PIDCoefficients(0.030, 0, 0.0);
        public static double PIDTolerance = 20; //Tolerance in TICKS
    }
}
