package org.firstinspires.ftc.teamcode.RobotModes.TeleOps;


import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LSx1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LSy1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.RSx1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.Y1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSub;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;

@TeleOp(name = "Solvers Mega Proof", group = "SolversLib")
public class SolversMegaProof extends OpMode {
    ControllerInitializer controllerInitializer;
    TelemetryMethods telemetryMethods;
    MecanumDriveSub mecanumDriveSub;
    //Camera_Detection cameraDetection;
    public static boolean fieldCentric = false;
    public static boolean oncePressed = false;
    Limelight limelight;

    @Override
    public void init() {
        controllerInitializer = new ControllerInitializer(gamepad1, gamepad2);
        telemetryMethods = new TelemetryMethods();
        mecanumDriveSub = new MecanumDriveSub(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }
    @Override
    public void loop() {
        controllerInitializer.ActualizeGamepad1();
        controllerInitializer.ActualizeGamepad2();
        limelight.getCameraBasicValues();
        limelight.getFiducialResults();
        limelight.getBotPose();
        mecanumDriveSub.getActualYaw();
        mecanumDriveSub.getActualVel();
        telemetryMethods.getChassisTelemetry(telemetry);


        if (Y1 == 1) {
            fieldCentric = !oncePressed ? !fieldCentric : fieldCentric;
            oncePressed = true;
        } else {
            mecanumDriveSub.driveRobotPOV(-LSx1, LSy1, -RSx1);
            oncePressed = false;
        }
    }

    @Override
    public void stop() {
        mecanumDriveSub.stopMotors();

    }

}
