package org.firstinspires.ftc.teamcode.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSub;

public class Dashboard {
    public static FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public static TelemetryPacket packet;
    
    ElapsedTime timer;
    double finalTime;

    ShooterSub shooterSub;
    MecanumDriveSub mecanumDriveSub;
    
    boolean isStable;

    // Variables para el tiempo de ciclo
    private int ballCount = 0;
    private boolean isshooting = false;
    private double cycleStartTime = 0;
    private double lastCycleTime = 0;
    private static final double RPM_DROP = 150;

    public Dashboard(){
        packet = new TelemetryPacket(false);

        shooterSub = SubsystemsInitializer.shooterSub;
        mecanumDriveSub = SubsystemsInitializer.mecanumDriveSub;
        // mecanumDriveSub.getAllChassisValues();
        timer = new ElapsedTime();
        timer.reset();
    }
    
    public void getStabilizationTime (){
        double time;
        while (shooterSub.desiredRPMs < shooterSub.getShooterVel()) {
            isStable = false;
            time = timer.seconds();
            packet.put("Stabilization Time", time);
            ftcDashboard.sendTelemetryPacket(packet);
        } 
        isStable = true;
        finalTime = timer.seconds();
        packet.put("Final Time", finalTime);
    }
    public void getterShooter(){
        packet.put("Desired Revs", shooterSub.desiredRPMs);
        packet.put("Actual Revs", shooterSub.shooterVel);
        packet.put("RPMs Error", shooterSub.rpmsError);
    }
    
    public void getChassisVel(){
        packet.put("Chassis Vel X", mecanumDriveSub.xVelocity);
        packet.put("Chassis Vel Y", mecanumDriveSub.yVelocity);
    }
    

    public void cicleTime(){
        double currentVel = shooterSub.getShooterVel();
        double targetVel = shooterSub.desiredRPMs;

        if (!isshooting && currentVel < (targetVel - RPM_DROP)) {
            isshooting = true;
            if (ballCount == 0) {
                cycleStartTime = timer.seconds();
            }
            ballCount++;
        } 
        
        else if (isshooting && currentVel > (targetVel - (RPM_DROP * 0.5))) {
            isshooting = false;
            
            if (ballCount >= 3) {
                lastCycleTime = timer.seconds() - cycleStartTime;
                ballCount = 0;
            }
        }

        packet.put("Pelotas detectadas", ballCount);
        packet.put("Tiempo de Ciclo", lastCycleTime);
        ftcDashboard.sendTelemetryPacket(packet);
    }
}
