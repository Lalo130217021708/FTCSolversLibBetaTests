package org.firstinspires.ftc.teamcode.Camera;

//import static rg.firstinspires.ftc.teamcode.Camera_Autonomous.cameraOrientation;
//import static org.firstinspires.ftc.teamcode.RobotModes.Autos.Autonomous.Camera_Autonomous.cameraPosition;


import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@SuppressWarnings({})
public class Camera_Detection{
    public Camera_Stream streamProcessor = new Camera_Stream();
    public AprilTagProcessor detectionProcessor;
    public VisionPortal visionPortal;
    public static double x;
    public static double y;
    public static double z;
    public static double yaw;
    public static double pitch;
    public static double roll;
    public static double bearing;
    public static double range;
    public static double elevation;
    public static double id;
    public static boolean detection;
    public static double xProof;
    public static double yProof;
    public static double zProof;
    public static double yawProof;
    public static double pitchProof;
    public static double rollProof;
    public static double bearingProof;
    public static double rangeProof;
    public static double elevationProof;
    public static double idProof;
    public static boolean detectionProof;

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>
            (Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public Camera_Detection(HardwareMap hardwareMap) {
        // We create the builder with our desired building for the AprilTag processor
        // and the VisionPortal

        //Set the orientation of the camera in the robot
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90, 0, 0);
        // Set the position of the camera in the robot
        Position cameraPosition = new Position(DistanceUnit.INCH,
                0, 0, 0, 0);

        detectionProcessor = new AprilTagProcessor.Builder()
                //Set Camera´s position and orientation in the robot
                .setCameraPose(cameraPosition, cameraOrientation)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                //Specify the april Tags that we will use this competition
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                //Specify the units we want to use for the output detections
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        

        //Initializing the visionPortal and its building process
         visionPortal = new VisionPortal.Builder()
                //Create our Camera using the hardwareMap
                .setCamera(hardwareMap.get(WebcamName.class, "WebCam"))
                 
                //We assign the aprilTagProcessor and visionProcessor (Used for Stream)
                .addProcessors(detectionProcessor, streamProcessor)
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(R.id.cameraMonitorViewId)
                .setAutoStartStreamOnBuild(true)
                .build();

    }
    public boolean setManualExposure(int exposureMS, int gain) {
        // 1. Verificamos si la cámara ya está transmitiendo. Si no, no podemos configurar nada.
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        // 2. Obtenemos el controlador de exposición
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        // 3. Cambiamos a modo MANUAL si no lo está ya
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

            // A veces se necesita un pequeño delay para que el hardware procese el cambio de modo
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // 4. Establecemos la exposición (en milisegundos)
        // Para AprilTags en movimiento, 4ms - 6ms es ideal.
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // 5. Ajustamos la Ganancia (Gain) para compensar la oscuridad de la baja exposición
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        return true; // Éxito
    }

    public void CameraDetectionRedGoal() {
        if (detectionProcessor.getDetections().isEmpty()) {
            detection = false;
        } else {
            for (AprilTagDetection detection : detectionProcessor.getDetections()) {
                // If the detection is the April Tag 24 (The one in the Red Goal), it will
                // mark the new values obtained
                if (detection.id == 24) {
                    id = detection.id;

                    //Getting xDistance, yDistance and zDistance
                    x = detection.ftcPose.x;
                    y = detection.ftcPose.y;
                    z = detection.ftcPose.z;

                    //Getting Yaw, Pitch and Roll, used on angulation/orientation
                    yaw = detection.ftcPose.yaw;
                    pitch = detection.ftcPose.pitch;
                    roll = detection.ftcPose.roll;

                    //Getting range, bearing and elevation
                    range = detection.ftcPose.range;
                    bearing = detection.ftcPose.bearing;
                    elevation = detection.ftcPose.elevation;
                }
            }
            detection = true;
        }
    }
    public void CameraDetectionBlueGoal() {
        if (detectionProcessor.getDetections().isEmpty()) {
            detection = false;
        } else {
            for (AprilTagDetection detection : detectionProcessor.getDetections()) {
                // If the detection is the April Tag 20 (The one in the Blue Goal), it will
                // mark the new values obtained
                if (detection.id == 20) {
                    id = detection.id;

                    //Getting xDistance, yDistance and zDistance
                    x = detection.ftcPose.x;
                    y = detection.ftcPose.y;
                    z = detection.ftcPose.z;

                    //Getting Yaw, Pitch and Roll, used on angulation/orientation
                    yaw = detection.ftcPose.yaw;
                    pitch = detection.ftcPose.pitch;
                    roll = detection.ftcPose.roll;

                    //Getting range, bearing and elevation
                    range = detection.ftcPose.range;
                    bearing = detection.ftcPose.bearing;
                    elevation = detection.ftcPose.elevation;
                }
            }
            detection = true;
        }
    }

    public void CameraDetection() {
        if (detectionProcessor.getDetections().isEmpty()) {
            detection = false;
        } else {
            for (AprilTagDetection detection : detectionProcessor.getDetections()) {
                // We put the detection values into the detectionValues array
                id = detection.id;

                //Getting xDistance, yDistance and zDistance
                x = detection.ftcPose.x;
                y = detection.ftcPose.y;
                z = detection.ftcPose.z;

                //Getting Yaw, Pitch and Roll, used on angulation/orientation
                yaw = detection.ftcPose.yaw;
                pitch = detection.ftcPose.pitch;
                roll = detection.ftcPose.roll;

                //Getting range, bearing and elevation
                range = detection.ftcPose.range;
                bearing = detection.ftcPose.bearing;
                elevation = detection.ftcPose.elevation;
            }
            detection = true;
        }
    }

    public void kindaMegaTag(){
        for (AprilTagDetection detection : detectionProcessor.getDetections()) {
            if(detection.robotPose != null){
                xProof = detection.robotPose.getPosition().x;
                yProof = detection.robotPose.getPosition().y;
                zProof = detection.robotPose.getPosition().z;
                yawProof = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            }
        }
    }
}