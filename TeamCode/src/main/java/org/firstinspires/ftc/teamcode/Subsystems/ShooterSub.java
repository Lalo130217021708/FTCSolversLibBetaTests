package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator.compensateVoltage;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class ShooterSub {
    private final Motor shooterMotor;
    public static double shooterCPR, shooterPos, shooterRate, shoterVel;
    public ShooterSub(HardwareMap hardwareMap) {
        shooterMotor = new Motor(hardwareMap, "shooterMotor");
        shooterMotor.setInverted(true);
        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(double power){
        shooterMotor.set(power);
    }

    public void shootClose(){
        shoot(compensateVoltage(.65));
    }

    public void shootFar(){
        shoot(compensateVoltage(.735));
    }

    public void getShooterMotorCPR(){
        shooterCPR = shooterMotor.getCPR();
    }
    public void getShooterPos(){
        shooterPos = shooterMotor.getCurrentPosition() / shooterMotor.getCPR();
    }
    public void getShooterRate(){
        shooterRate = shooterMotor.getRate();
    }

    public void getShooterVel(){
        shoterVel = shooterMotor.getRate() * shooterMotor.getCPR() * .006;
    }

    public void getGetters(){
        getShooterMotorCPR();
        getShooterPos();
        getShooterRate();
        getShooterVel();
    }
    public void stop(){
        shooterMotor.set(0);
    }
}
