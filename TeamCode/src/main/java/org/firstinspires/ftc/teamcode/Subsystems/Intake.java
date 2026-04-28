package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake {
    private Motor intakeMotor;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }
}
