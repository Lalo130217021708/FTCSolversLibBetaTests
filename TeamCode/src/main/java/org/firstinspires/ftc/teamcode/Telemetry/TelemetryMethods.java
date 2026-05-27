package org.firstinspires.ftc.teamcode.Telemetry;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.StrafeDistance_3D;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.distance;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.fidTY;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.ta;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tagCount;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tx;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.ty;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.x;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.xBotPose;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.y;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.yBotPose;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.configurableRPMs;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.fieldCentric;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.kkd;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.kkf;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.kki;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.kkp;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.actualYaw;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.pos;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.vel;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.interpLUT;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterCPR;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterPos;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterRate;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel;
import static org.firstinspires.ftc.teamcode.Subsystems.ShooterSub.shooterVel2;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Limelight;

public class TelemetryMethods {
    Telemetry telemetry;
    public TelemetryMethods(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void getRobotTelemetry() {
        getLimelightValues();
    }

    public void getTel(){
        telemetry.addData("P", kkp);
        telemetry.addData("I", kki);
        telemetry.addData("D", kkd);
        telemetry.addData("F", kkf);
    }
    public void getChassisTelemetry(){
        telemetry.addData("Heading" ,actualYaw);
        telemetry.addData("FieldCentric", fieldCentric);
    }
    public void getOdometryTelemetry() {
        telemetry.addData("X", pos[2]);
        telemetry.addData("YLeft", pos[0]);
        telemetry.addData("YRight", pos[1]);
        telemetry.addData("Heading", actualYaw);
    }
    public void getLimelightValues(){
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.addData("tagCount", tagCount);
        telemetry.addData("distance to Goal", distance);
        telemetry.addData("fidTY", fidTY);
        telemetry.addData("intplut", interpLUT.get(distance));
    }

    public void getLimelight(){
        telemetry.addData("Strafe Distance", StrafeDistance_3D);
        telemetry.addData("x Distance Relative Tag", x);
        telemetry.addData("y Distance Relative Tag", y);
        telemetry.addData("Tag Id", Limelight.id);

    }

    public void getBotPose(){
        telemetry.addData("X Bot Pose", xBotPose);
        telemetry.addData("Y Bot Pose", yBotPose);
    }
    public void getShooterValues(){
        telemetry.addData("Shooter CPR", shooterCPR);
        telemetry.addData("Shooter Pos", shooterPos);
        telemetry.addData("Shooter Rate", shooterRate);
        telemetry.addData("Shooter Vel", shooterVel);
        telemetry.addData("Shooter Vel 2", shooterVel2);
        telemetry.addData("rpms", configurableRPMs);
    }
}
