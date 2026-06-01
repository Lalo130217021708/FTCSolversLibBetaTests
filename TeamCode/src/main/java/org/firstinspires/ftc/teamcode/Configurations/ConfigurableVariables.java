package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;

public class ConfigurableVariables {
    @Config
    public static class shooterConfigurableVariables{
        public static double p = .025;
        public static double i = 1.25;
        public static double d = 0.05;
        public static double f = 0;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class ExperimentalVariables{
    }

}
