package org.firstinspires.ftc.teamcode.Telemetry;

import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.bearing;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.id;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.range;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.xProof;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.yProof;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.yawProof;
import static org.firstinspires.ftc.teamcode.Camera.Camera_Detection.zProof;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.StrafeDistance_3D;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.ta;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tagCount;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.tx;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.ty;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.x;
import static org.firstinspires.ftc.teamcode.Camera.Limelight.y;
import static org.firstinspires.ftc.teamcode.RobotModes.TeleOps.SolversMegaProof.fieldCentric;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.actualYaw;
import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.vel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Limelight;

public class TelemetryMethods {
    public void getRobotTelemetry(Telemetry telemetry) {
        getChassisTelemetry(telemetry);
        getLimelightValues(telemetry);
        getLimelight(telemetry);
    }
    public void getChassisTelemetry(Telemetry telemetry){
        telemetry.addData("FLVel", vel[0]);
        telemetry.addData("FRVel", vel[1]);
        telemetry.addData("RLVel", vel[2]);
        telemetry.addData("RRVel", vel[3]);
        telemetry.addData("Heading" ,actualYaw);
        telemetry.addData("FieldCentric", fieldCentric);
        telemetry.update();
    }
    public void getLimelightValues(Telemetry telemetry){
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.addData("tagCount", tagCount);
    }

    public void getLimelight(Telemetry telemetry){
        telemetry.addData("Strafe Distance", StrafeDistance_3D);
        telemetry.addData("x Distance Relative Tag", x);
        telemetry.addData("y Distance Relative Tag", y);
        telemetry.addData("Tag Id", Limelight.id);
    }
}
