package org.firstinspires.ftc.teamcode.Initializers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double time;
    public static boolean onceSaved = false;
    private final ElapsedTime timer = new ElapsedTime();



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
        if (Math.abs(shooterSub.desiredRPMs - shooterSub.getShooterVel()) < 20) {
            if(!onceSaved) {
                time = timer.seconds();
                onceSaved = true;
            }
            intakeFeederSub.intake(1);
            intakeFeederSub.feeder(1);
        } else {
            timer.reset();
            intakeFeederSub.stop();
        }
     }

}
