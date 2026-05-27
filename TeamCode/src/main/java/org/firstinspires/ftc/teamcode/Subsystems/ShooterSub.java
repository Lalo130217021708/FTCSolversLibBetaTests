package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.distance;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;

public class ShooterSub {
    private final MotorEx shooterMotor, shooterMotor2;
    public static double shooterCPR, shooterPos, shooterRate, shooterVel, shooterVel2;
    private final MotorGroup shooterMotors;
    public static InterpLUT interpLUT;
    private PIDFCoefficients pidfCoefficients;
    private PIDFController pid;

    private VoltageCompensator voltageCompensator;
    public double desiredRPMS;
    public ShooterSub(HardwareMap hardwareMap, VoltageCompensator voltageCompensator) {
        shooterMotor = new MotorEx(hardwareMap, "shooterMotor");
        shooterMotor.setInverted(false);
        shooterMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotor2 = new MotorEx(hardwareMap, "shooterMotor2");
        shooterMotor2.setInverted(true);
        shooterMotor2.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotors = new MotorGroup(shooterMotor, shooterMotor2);

        interpLUT = new InterpLUT();

        this.voltageCompensator = voltageCompensator;
        setInterpLUTValues();
    }
    /// Shooter Functions ///
    public void shootSingleRight(double power){
        shooterMotor.set(power);
    }
    public void shootSingleLeft(double power){shooterMotor2.set(power);}
    public void shootRPMs(){
        pid = new PIDFController(.00215, 1.25, .1, voltageCompensator.compensateVoltage(.000185));
        shooterMotors.set(pid.calculate(shooterVel, desiredRPMS));
    }
    public void shootManually(double power){shooterMotors.set(power);}
    public void stop(){shooterMotors.set(0);}

    /// Shooter Getters
    public void getShooterMotorCPR(){shooterCPR = shooterMotor.getCPR();}
    public void getShooterPos(){shooterPos = shooterMotor.getCurrentPosition() / shooterMotor.getCPR();}
    public void getShooterRate(){shooterRate = shooterMotor.getRate();}
    public double getShooterVel(){
        shooterVel = shooterMotor.getVelocity()/28 * 60;
        return shooterVel;
    }
    public void getInterpLUT(){
        desiredRPMS = interpLUT.get(distance);
    }
    public double getShooterVel2(){
        shooterVel2 = shooterMotor2.getVelocity()/28 * 60;
        return shooterVel2;
    }
    public void getGetters(){
        getShooterMotorCPR();
        getShooterPos();
        getShooterRate();
        getShooterVel();
        getShooterVel2();
    }
    /// Setters ///
    public void setInterpLUTValues(){
        interpLUT.add(0, 3085);
        interpLUT.add(27.2, 3085);
        interpLUT.add(45.3, 3195);
        interpLUT.add(77.94, 3600);
        interpLUT.add(86.5, 3771);
        interpLUT.add(111.85, 4157);
        interpLUT.add(116.1, 4328);
        interpLUT.add(136.1, 4500);

        interpLUT.createLUT();
    }
}
