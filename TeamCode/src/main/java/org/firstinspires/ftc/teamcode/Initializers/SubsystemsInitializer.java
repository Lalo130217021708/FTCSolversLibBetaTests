package org.firstinspires.ftc.teamcode.Initializers;

import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.DesiredRPMs;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSub;

import java.util.Timer;

public class SubsystemsInitializer {
    public static MecanumDriveSub mecanumDriveSub;
    public static IntakeFeederSub intakeFeederSub;
    public static ShooterSub shooterSub;
    public static VoltageCompensator voltageCompensator;
    public static Limelight limelight;
    public static boolean onceSaved = false;
    public static boolean once = false;
    public static double time = 0;
    ElapsedTime timer = new ElapsedTime();

    public SubsystemsInitializer(HardwareMap hardwareMap){
        intakeFeederSub = new IntakeFeederSub(hardwareMap);
        voltageCompensator = new VoltageCompensator(hardwareMap);
        shooterSub = new ShooterSub(hardwareMap, voltageCompensator);
        limelight = new Limelight(hardwareMap);
        mecanumDriveSub = new MecanumDriveSub(hardwareMap,limelight);

    }

    public void absStop(){
        mecanumDriveSub.stopMotors();
        intakeFeederSub.stop();
    }

    public double getCompensationTimer(){
        if (Math.abs(DesiredRPMs - shooterVel) > 300) {
            if (!onceSaved) {
                time = timer.seconds();
                onceSaved = true;
            }
        } else {
            timer.reset();
        }
        return time;
    }
     public void automatizedShoot(){
        shooterSub.shootRPMs();
        if (Math.abs(DesiredRPMs -shooterVel) < 50) {
            intakeFeederSub.intake(1);
            intakeFeederSub.feeder(1);
        } else {
            intakeFeederSub.intake(0.55);
            intakeFeederSub.feeder(0.55);
        }
     }

}
