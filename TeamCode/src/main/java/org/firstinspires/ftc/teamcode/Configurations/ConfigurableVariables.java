package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;

public class ConfigurableVariables {
    @Config
    public static class shooterConfigurableVariables{
        public static double pShooter = 0.0015;
        public static double iShooter = .75;
        public static double dShooter = .125;
        public static double fShooter = 0.00019;
        public static double p = 0.03;
        public static double i = .5;
        public static double d = .925;
        public static double f = 0;
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
        public static double configurableRPMs = 3085;
    }

    public static class ExperimentalVariables{
    }

}
