package org.firstinspires.ftc.teamcode.Configurations;

import com.seattlesolvers.solverslib.geometry.Translation2d;

public class DriveConstants {
    public static class WheelsPoses{
        //Max 17.325 Y reference on 2.5
        //Max 18 X reference on 2
        public static Translation2d frontLeftPose = new Translation2d(-7,6.1625);
        public static Translation2d frontRightPose = new Translation2d(7,6.1625);
        public static Translation2d rearLeftPose = new Translation2d(-7,-6.1625);
        public static Translation2d rearRightPose = new Translation2d(7,-6.1625);

    }
}
