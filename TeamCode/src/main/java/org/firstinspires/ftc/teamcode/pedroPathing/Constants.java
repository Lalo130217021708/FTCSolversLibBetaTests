package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Constants {

    public static double brakingStrenght = .2;
    public static double brakingStart = .175;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-24.530167827548517)
            .lateralZeroPowerAcceleration(-45.1642393813147)
            .headingPIDFCoefficients(new PIDFCoefficients(1.25, 2.5, 0.05, 0.003))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.6125,0,0.025,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.015,.00,.00,0,0))
            .centripetalScaling(.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.9)
            .xVelocity(58.536941950990276)
            .yVelocity(43.88946861258117)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("rearRight")
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("rearLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.00296843400339)
            .strafeTicksToInches(0.00296843400339)
            .turnTicksToInches(-.00296843400339)
            .strafePodX(-5.7)
            .leftPodY(8)
            .rightPodY(-8)
            .leftEncoder_HardwareMapName("frontLeft")
            .rightEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("rearLeft")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            brakingStrenght,
            brakingStart
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }

    public static double getBrakingStrenght(){
        return pathConstraints.getBrakingStrength();
    }
}