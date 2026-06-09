package org.firstinspires.ftc.teamcode.Telemetry;

import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.DesiredRPMs;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;

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

    ElapsedTime timer2;
    double finalTime;

    ShooterSub shooterSub;
    MecanumDriveSub mecanumDriveSub;
    
    boolean isStable;

    double time;

    // Variables para el tiempo de ciclo
    private int ballCount = 0;
    private boolean isshooting = false;
    private double cycleStartTime = 0;
    private double lastCycleTime = 0;
    private static final double RPM_DROP = 150;
    private boolean onceSaved = false;
    private boolean onceReseted = false;

    public Dashboard(){
        packet = new TelemetryPacket(false);

        shooterSub = SubsystemsInitializer.shooterSub;
        mecanumDriveSub = SubsystemsInitializer.mecanumDriveSub;
        // mecanumDriveSub.getAllChassisValues();
        timer = new ElapsedTime();
        timer.reset();
    }
    
    public void getStabilizationTime (){

        //if (shooterSub.getShooterVel() - 100 < 2200 ||  shooterSub.getShooterVel() + 100 < 2300 ||


        ftcDashboard.sendTelemetryPacket(packet);

    }
    public void getterShooter(){
        packet.put("Desired Revs", DesiredRPMs);
        packet.put("Actual Revs", shooterSub.shooterVel);
        packet.put("RPMs Error", shooterSub.rpmsError);

        ftcDashboard.sendTelemetryPacket(packet);
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


    public void printpene (double pene){
        packet.put("Time", pene);
        ftcDashboard.sendTelemetryPacket(packet);
    }
}
