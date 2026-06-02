package org.firstinspires.ftc.teamcode.Initializers;

import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.configurableRPMs;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSub;

public class SubsystemsInitializer {
    public static MecanumDriveSub mecanumDriveSub;
    public static IntakeFeederSub intakeFeederSub;
    public static ShooterSub shooterSub;
    public static VoltageCompensator voltageCompensator;
    public static Limelight limelight;
    public static boolean onceSaved = false;

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
     public void automatizedShoot(){
        shooterSub.shootRPMs();
        if (Math.abs(configurableRPMs - shooterSub.getShooterVel()) < 25) {
            intakeFeederSub.intake(1);
            intakeFeederSub.feeder(1);
        } else {
             intakeFeederSub.stop();
         }
     }

}
