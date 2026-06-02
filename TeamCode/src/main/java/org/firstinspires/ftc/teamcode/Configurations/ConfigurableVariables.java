package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;

public class ConfigurableVariables {
    @Config
    public static class shooterConfigurableVariables{
        public static double p = 0.0015;
        public static double i = .75;
        public static double d = .125;
        public static double f = 0.000197;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
        public static double configurableRPMs = 3085;
    }

    public static class ExperimentalVariables{
    }

}
