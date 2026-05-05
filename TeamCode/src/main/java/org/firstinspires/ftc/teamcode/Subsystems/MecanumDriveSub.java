package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.tx;
import static org.firstinspires.ftc.teamcode.Configurations.DriveConstants.WheelsPoses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSub {
    /// Variables
    public static double actualYaw;
    public static double[] vel = new double[4];
    public static double[] pos = new double[3];
    boolean onceSaved = false;
    double zOutput;
    double savedYaw;


    /// Drive Bases Controllers Creators and Initializers
    MecanumDrive mecanumDrive;
    MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(
            WheelsPoses.frontLeftPose,
            WheelsPoses.frontRightPose,
            WheelsPoses.rearLeftPose,
            WheelsPoses.rearRightPose
    );

    /// Hardware Creators
    Motor frontLeftMotor;
    Motor frontRightMotor;
    Motor rearLeftMotor;
    Motor rearRightMotor;
    IMU imu;

    /// PIDFControllers and Coefficients Creators
    PIDFController yawController;
    PIDFCoefficients pidfYawCoefficients;
    PIDFController aprilTagController;
    PIDFCoefficients pidfAtCoefficients;

    public MecanumDriveSub(HardwareMap hardwareMap) {
        /// Motor Getters and Configurators
        frontLeftMotor = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRightMotor = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearLeftMotor = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        rearRightMotor = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(false);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(false);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /// Imu Getter
        imu = hardwareMap.get(IMU.class, "imu");


        mecanumDrive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                rearLeftMotor,
                rearRightMotor
        );

        pidfYawCoefficients = new PIDFCoefficients(0.03,0.0,0.0,0.0);
        pidfAtCoefficients = new PIDFCoefficients(0.032,0.0,0.0,0.0);

        yawController = new PIDFController(pidfYawCoefficients);
        yawController.setTolerance(1);

        aprilTagController = new PIDFController(pidfAtCoefficients);
        aprilTagController.setTolerance(.25);
    }


    /// Chassis Functions
    public void driveDriverPOV(double xInput, double yInput, double zInput){
        savedYaw = actualYaw;
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.toFieldRelativeSpeeds(
                new ChassisSpeeds(
                    xInput,
                    yInput,
                    zInput
                ),
                getHeading()
        );

        MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(chassisSpeeds);

        frontLeftMotor.set(-mecanumDriveWheelSpeeds.frontLeftMetersPerSecond);
        frontRightMotor.set(-mecanumDriveWheelSpeeds.frontRightMetersPerSecond);
        rearLeftMotor.set(mecanumDriveWheelSpeeds.rearLeftMetersPerSecond);
        rearRightMotor.set(mecanumDriveWheelSpeeds.rearRightMetersPerSecond);
    }
    public void driveRobotPOV(double xInput, double yInput, double zInput){
        if(zInput == 0){
            if(!onceSaved){
                savedYaw = actualYaw;
                onceSaved = true;
            }
            zOutput = yawController.calculate(actualYaw ,savedYaw);
        } else {
            zOutput = zInput;
            onceSaved = false;
        }
        mecanumDrive.driveRobotCentric(xInput, yInput, zOutput);
    }
    public void driveRobot(boolean fieldCentric, double xInput, double yInput, double zInput){
        if (fieldCentric) {
            driveDriverPOV(xInput, yInput, zInput);
        } else {
            driveRobotPOV(-xInput, yInput, -zInput);
        }
    }
    public void aprilTagTracking(double xInput, double yInput){
        double zCalculations = aprilTagController.calculate(tx, 0);
        driveDriverPOV(xInput, yInput, -zCalculations);
    }
    public void stopMotors(){
        frontLeftMotor.set(0);
        frontRightMotor.set(0);
        rearLeftMotor.set(0);
        rearRightMotor.set(0);
    }


    /// Getters
    public void getActualVel(){
        vel[0] = frontLeftMotor.getRawPower();
        vel[1] = frontRightMotor.getRawPower();
        vel[2] = rearLeftMotor.getRawPower();
        vel[3] = rearRightMotor.getRawPower();
    }
    public void getActualPos(){
        pos[0] = (frontLeftMotor.getCurrentPosition() / frontLeftMotor.getCPR()) / .5936856133;
        pos[1] = (frontRightMotor.getCurrentPosition() / frontLeftMotor.getCPR()) / .5936856133;
        pos[2] = (rearLeftMotor.getCurrentPosition() / frontLeftMotor.getCPR()) / .5936856133;
    }
    public void getActualYaw(){
        actualYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
    public void getAllChassisValues(){
        getHeading();
        getActualVel();
        getActualPos();
        getActualYaw();
    }


    /// Reseters
    public void resetYaw(){
        imu.resetYaw();
    }
    public void resetEncoder() {
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
    }
     public void resetVar(){
        pos[0] = 0;
        pos[1] = 0;
        pos[2] = 0;
        savedYaw = 0;
        onceSaved = false;
     }
     public void resetAllChassisValues(){
        resetVar();
        resetEncoder();
        resetYaw();
     }
}
