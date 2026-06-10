package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import java.util.Collections;
import java.util.Set;

public class IntakeFeederSub {
    private final Motor intakeMotor;
    private final Motor feederMotor;

    public IntakeFeederSub(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        feederMotor = new Motor(hardwareMap, "feederMotor");
        intakeMotor.setInverted(true);
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
    public Command intakeFeederCdm = new Command() {

        @Override
        public Set<Object> requirements() {
            return Collections.emptySet();
        }

        @Override
        public int priority() {
            return 0;
        }

        @Override
        public InterruptedBehavior interruptedBehavior() {
            return null;
        }

        @Override
        public ConflictBehavior conflictBehavior() {
            return null;
        }

        @Override
        public BlockedBehavior blockedBehavior() {
            return null;
        }

        @Override
        public void start() {

        }

        @Override
        public boolean done() {
            return false;
        }

        @Override
        public void execute() {
            intake(1);
            feeder(.2);
        }

        @Override
        public void end(EndCondition endCondition) {
            intake(0);
            feeder(0);
        }
    };

}
