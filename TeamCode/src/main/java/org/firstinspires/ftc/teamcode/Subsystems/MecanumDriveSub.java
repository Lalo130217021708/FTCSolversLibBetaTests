package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.id;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tx;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.d;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.f;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.i;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.p;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.Limelight;

public class MecanumDriveSub {
    /// Variables
    public static double actualYaw;
    public static double[] vel = new double[4];
    public static double[] pos = new double[3];
    boolean onceSaved = false;
    public static double setPoint, yVelocity, xVelocity;


    /// Drive Bases Controllers Creators and Initializers
    MecanumDrive mecanumDrive;

    /// Hardware Creators
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx rearLeftMotor;
    private final MotorEx rearRightMotor;
    IMU imu;

    /// PIDFControllers and Coefficients Creators
    PIDFController aprilTagController;

    private final Limelight limelight;
    public MecanumDriveSub(HardwareMap hardwareMap, Limelight limelight) {
        /// Motor Getters and Configurators
        frontLeftMotor = new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);
        rearRightMotor = new MotorEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);


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

        /// Utilities initialized
        mecanumDrive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                rearLeftMotor,
                rearRightMotor
        );

        this.limelight = limelight;
    }


    /// Chassis Functions
    public void driveRobotPOV(double xInput, double yInput, double zInput){
            mecanumDrive.driveRobotCentric(-xInput, yInput, -zInput);
    }
    public void aprilTagTracking(double xInput, double yInput, double zInput){
        aprilTagController = new PIDFController(p,i,d,f);
        setPoint = limelight.getGoalSetPoint();

        double zCalculations = id == 20 || id == 24 ? aprilTagController.calculate(tx, -setPoint) : zInput;
        driveRobotPOV(xInput*.5, yInput*.5, -zCalculations);
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
    public void getHeading(){
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void getVelocity(){
        xVelocity = rearLeftMotor.getVelocity() * 0.00296843400339;
        yVelocity = ((frontLeftMotor.getVelocity() + frontRightMotor.getVelocity()) / 2) * 0.00296843400339;
    }
    public void getAllChassisValues(){
        getVelocity();
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
        onceSaved = false;
     }
     public void resetAllChassisValues(){
        resetVar();
        resetEncoder();
        resetYaw();
     }
}
