package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeFeederSub {
    private DcMotorEx intakeMotor;
    private DcMotorEx feederMotor;
    public IntakeFeederSub(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        feederMotor = hardwareMap.get(DcMotorEx.class, "feederMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        feederMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void intake(double power) {
        intakeMotor.setPower(power);
    }
    public void feeder(double power) {
        feederMotor.setPower(power);
    }
    public void stop(){
        intakeMotor.setPower(0);
        feederMotor.setPower(0);
    }
}
