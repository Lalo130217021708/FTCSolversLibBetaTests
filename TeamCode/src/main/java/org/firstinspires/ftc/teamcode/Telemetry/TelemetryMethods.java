package org.firstinspires.ftc.teamcode.Telemetry;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.distance;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.fidTY;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tx;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.xBotPose;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.yBotPose;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.setPoint;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.xVelocity;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.yVelocity;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.interpLUT;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryMethods {
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    TelemetryPacket telemetryPacket = new TelemetryPacket(false);
    public Telemetry dashTelemetry = ftcDashboard.getTelemetry();
    Telemetry telemetry;
    public TelemetryMethods(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void getRobotTelemetry() {
        getLimelightValues();
        getShooterValues();
        getBotPose();
        getChassisVel();
        ftcDashboard.sendTelemetryPacket(telemetryPacket);
    }
    public void getChassisVel(){
        telemetryPacket.put("xVel", xVelocity);
        telemetryPacket.put("yVel", yVelocity);
        telemetry.addData("xVel", xVelocity);
        telemetry.addData("yVel", yVelocity);
    }
    public void getLimelightValues(){
        telemetry.addData("tx", tx);
        telemetry.addData("distance to Goal", distance);
        telemetry.addData("fidTY", fidTY);
        telemetry.addData("intplut", interpLUT.get(distance));
    }

    public void getBotPose(){
        telemetry.addData("X Bot Pose", xBotPose);
        telemetry.addData("Y Bot Pose", yBotPose);
    }
    public void getShooterValues(){
        telemetry.addData("Shooter Vel", shooterVel);
        telemetry.addData("Shooter Vel 2", shooterVel2);
        telemetry.addData("setpoint", setPoint);
    }
}
