package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.configurablePower;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.configurableRPMs;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.d;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.f;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.i;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.p;
import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSub {
    private final MotorEx shooterMotor, shooterMotor2;
    public static double shooterCPR, shooterPos, shooterRate, shooterVel, shooterVel2;
    private final MotorGroup shooterMotors;
    private final InterpLUT interpLUT;
    private PIDFCoefficients pidfCoefficients;
    private PIDFController pid;

    public ShooterSub(HardwareMap hardwareMap) {
        shooterMotor = new MotorEx(hardwareMap, "shooterMotor");
        shooterMotor.setInverted(true);
        shooterMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotor2 = new MotorEx(hardwareMap, "shooterMotor2");
        shooterMotor2.setInverted(false);
        shooterMotor2.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotors = new MotorGroup(shooterMotor, shooterMotor2);

        interpLUT = new InterpLUT();

        pidfCoefficients = new PIDFCoefficients(0.00065, 0.95, 0.005, 0.000197);
    }
    /// Shooter Functions ///
    public void shootSingleRight(double power){
        shooterMotor.set(power);
    }
    public void shootSingleLeft(double power){shooterMotor2.set(power);}
    public void configShoot(){shooterMotors.set(configurablePower);}
    public void shootRPMs(){
        pid = new PIDFController(p, i , d, f);
        shooterMotors.set(pid.calculate(shooterVel, configurableRPMs));
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
        interpLUT.add(0, 0);
        interpLUT.add(1, 1);
        interpLUT.add(2, 2);
        interpLUT.add(3, 3);
        interpLUT.add(4, 4);
        interpLUT.add(5, 5);
        interpLUT.add(6, 6);
        interpLUT.add(7, 7);
        interpLUT.add(8, 8);
        interpLUT.add(9, 9);
        interpLUT.add(10, 10);
    }
}
