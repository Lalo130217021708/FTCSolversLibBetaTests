package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class IntakeFeederSub {
    private final Motor intakeMotor;
    private final Motor feederMotor;

    public IntakeFeederSub(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        feederMotor = new Motor(hardwareMap, "feederMotor");
        intakeMotor.setInverted(false);
        feederMotor.setInverted(false);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(double power) {
        intakeMotor.set(power);
    }
    public void feeder(double power) {
        feederMotor.set(power);
    }
    public void stop(){
        intakeMotor.set(0);
        feederMotor.set(0);
    }
}
