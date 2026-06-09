package org.firstinspires.ftc.teamcode.Initializers;

import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSub;

import java.util.Collections;
import java.util.Set;

public class SubsystemsInitializer {
    public static MecanumDriveSub mecanumDriveSub;
    public static IntakeFeederSub intakeFeederSub;
    public static ShooterSub shooterSub;
    public static VoltageCompensator voltageCompensator;
    public static Limelight limelight;
    public static boolean onceSaved = false;
    public static boolean once = false;
    ElapsedTime elapsedTime;

    public SubsystemsInitializer(HardwareMap hardwareMap){
        intakeFeederSub = new IntakeFeederSub(hardwareMap);
        voltageCompensator = new VoltageCompensator(hardwareMap);
        shooterSub = new ShooterSub(hardwareMap, voltageCompensator);
        limelight = new Limelight(hardwareMap);
        mecanumDriveSub = new MecanumDriveSub(hardwareMap,limelight);
        elapsedTime = new ElapsedTime();

    }

    public void absStop(){
        mecanumDriveSub.stopMotors();
        intakeFeederSub.stop();
    }
     public void automatizedShoot(){
        shooterSub.shootRPMs();
        if (Math.abs(shooterSub.desiredRPMs-shooterVel) < 65) {
            intakeFeederSub.intake(1);
            intakeFeederSub.feeder(1);
            once = true;
        } else if (true){
            intakeFeederSub.intake(0.55);
            intakeFeederSub.feeder(0.55);
        } else {
            once = false;
            intakeFeederSub.intake(0);
            intakeFeederSub.feeder(0);
        }
     }
     public void activateShooter(){
        shooterSub.shootRPMs();
     }

     public Command automatizedShootCmd  = new Command() {

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
            elapsedTime.reset();
         }

         @Override
         public boolean done() {
             if(elapsedTime.seconds() > 3.6){
                 return true;
             } else {
                 return false;
             }

         }

         @Override
         public void execute() {
             limelight.getLimeValues();
             limelight.getDistanceToAt();
             shooterSub.getGetters();
             shooterSub.getInterpLUT();
             automatizedShoot();
         }

         @Override
         public void end(EndCondition endCondition) {
            shooterSub.stop();
            intakeFeederSub.stop();
         }
     };
}
