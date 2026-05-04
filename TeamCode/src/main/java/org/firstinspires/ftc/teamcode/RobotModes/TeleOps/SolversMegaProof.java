package org.firstinspires.ftc.teamcode.RobotModes.TeleOps;


import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.A1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.B2;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LSx1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LSy1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LSy2;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.LT2;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.RSx1;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.RSy2;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.RT2;
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.Y1;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.limelight;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.mecanumDriveSub;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.shooterSub;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer;
import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;

@TeleOp(name = "Solvers Mega Proof", group = "SolversLib")
public class SolversMegaProof extends OpMode {
    ControllerInitializer controllerInitializer;
    TelemetryMethods telemetryMethods;
    SubsystemsInitializer subsystemsInitializer;

    public static boolean fieldCentric = false;
    public static boolean oncePressed = false;

    @Override
    public void init() {
        controllerInitializer = new ControllerInitializer(gamepad1, gamepad2);
        telemetryMethods = new TelemetryMethods(telemetry);
        subsystemsInitializer = new SubsystemsInitializer(hardwareMap);
    }

    @Override
    public void init_loop() {
        mecanumDriveSub.resetAllChassisValues();
    }

    @Override
    public void loop() {
        controllerInitializer.actualizeGamepad();
        limelight.getLimeValues();
        telemetryMethods.getRobotTelemetry();
        mecanumDriveSub.getAllChassisValues();
        shooterSub.getGetters();
        telemetryMethods.getShooterValues();


        if (Y1 == 1) {
            fieldCentric = !oncePressed ? !fieldCentric : fieldCentric;
            oncePressed = true;
        } else if (A1 == 1) {
            mecanumDriveSub.aprilTagTracking(LSx1, LSy1);
        } else {
            mecanumDriveSub.driveRobot(fieldCentric, LSx1, LSy1, RSx1);
            oncePressed = false;
        }

        if (B2 == 1) {
            intakeFeederSub.stop();
        } else {
            intakeFeederSub.intake(LSy2);
            intakeFeederSub.feeder(RSy2);
        }

        if (B2 == 2) {
            shooterSub.stop();
        } else {
            shooterSub.shoot(LT2 > .05 ? -LT2 : RT2);
        }
    }

    @Override
    public void stop() {
        mecanumDriveSub.stopMotors();

    }

}
