package org.firstinspires.ftc.teamcode.Initializers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSub;

public class SubsystemsInitializer {
    public static MecanumDriveSub mecanumDriveSub;
    public static IntakeFeederSub intakeFeederSub;
    public static ShooterSub shooterSub;
    public static Limelight limelight;


    public SubsystemsInitializer(HardwareMap hardwareMap){
        mecanumDriveSub = new MecanumDriveSub(hardwareMap);
        intakeFeederSub = new IntakeFeederSub(hardwareMap);
        shooterSub = new ShooterSub(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }

    public void absStop(){
        mecanumDriveSub.stopMotors();
        intakeFeederSub.stop();
    }

}
