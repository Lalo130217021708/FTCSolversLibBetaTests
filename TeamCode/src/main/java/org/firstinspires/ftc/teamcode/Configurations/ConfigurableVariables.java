package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;

public class ConfigurableVariables {
    @Config
    public static class shooterConfigurableVariables{
        public static double configurablePower = .65;
        public static double configurableRPMs = 4000;
        public static double p = .0;
        public static double i = 0;
        public static double d = 0;
        public static double f = 0;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }
}
