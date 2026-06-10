package org.firstinspires.ftc.teamcode.Camera;

import static org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub.actualYaw;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight {
    Limelight3A limelight;
    public static double xBotPose, yBotPose;

    public static double tx, ty, ta;
    public static int tagCount;
    public static double StrafeDistance_3D;
    public static int id;
    public static double x, y, distance, fidTY;
    LLResult result;

    public static final double CAMERA_HEIGHT = 12.75;
    public static final double GOAL_HEIGHT = 29.5;


    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void getLimeValues(){
        getBotPose();
        getCameraBasicValues();
        getDistanceToAt();
        getFiducialResults();
    }

    public void getBotPose(){

        result = limelight.getLatestResult();
        limelight.updateRobotOrientation(actualYaw);
        if(result != null && result.isValid() && (id == 20 || id == 24)){
            Pose3D mt1 = result.getBotpose();
            if(mt1 != null){
                xBotPose = mt1.getPosition().x / .0254;
                yBotPose = mt1.getPosition().y / .0254;
            }

        } else {
            xBotPose = 0;
            yBotPose = 0;
        }
    }

    public void getCameraBasicValues(){
        result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            tagCount = result.getBotposeTagCount();
        } else {
            tx = 0;
            ty = 0;
            ta = 0;
            tagCount = 0;
        }
    }

    public void getDistanceToAt(){
        distance = (GOAL_HEIGHT - CAMERA_HEIGHT) / tan(Math.toRadians(fidTY + 20));
    }

    public void getFiducialResults(){
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null ) {
            try{
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    id = fiducial.getFiducialId(); // The ID number of the fiducial
                    x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                    y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                    StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
                    fidTY = fiducial.getTargetYDegrees();

                }
            } catch (Exception e) {
                throw new RuntimeException("Oye Check This" + e.getMessage());
            }
        } else {
            id = 0;
            x = 0;
            y = 0;
            StrafeDistance_3D = 0;
        }
    }

    public double getGoalSetPoint(){
        double x_GOAL_POSE = -74;
        double xPose = x_GOAL_POSE - xBotPose;
        double y_GOAL_POSE = -74;
        double yPose = y_GOAL_POSE - yBotPose;

        double tan = xPose / yPose;
        return Math.atan(tan);

    }

}
