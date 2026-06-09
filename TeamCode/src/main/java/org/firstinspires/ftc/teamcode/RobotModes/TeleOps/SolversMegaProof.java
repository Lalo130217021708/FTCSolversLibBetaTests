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
import static org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer.Y2;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.limelight;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.mecanumDriveSub;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.onceSaved;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.once;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.shooterSub;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Initializers.ControllerInitializer;
import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Telemetry.Dashboard;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;

@TeleOp(name = "Solvers Mega Proof", group = "SolversLib")
public class SolversMegaProof extends OpMode {
    ControllerInitializer controllerInitializer;
    TelemetryMethods telemetryMethods;

    Dashboard dashboard;
    SubsystemsInitializer subsystemsInitializer;
    ElapsedTime timer;

    public static boolean fieldCentric = false;
    public static boolean oncePressed = false;
    public double time;
    Follower follower;

    @Override
    public void init() {
        controllerInitializer = new ControllerInitializer(gamepad1, gamepad2);
        //telemetryMethods = new TelemetryMethods(telemetry);
        subsystemsInitializer = new SubsystemsInitializer(hardwareMap);
        follower = createFollower(hardwareMap);
        dashboard = new Dashboard();
        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        mecanumDriveSub.resetAllChassisValues();
    }

    @Override
    public void loop() {
        controllerInitializer.actualizeGamepad();
        limelight.getLimeValues();
        limelight.getBotPose();
        mecanumDriveSub.getAllChassisValues();
        shooterSub.getGetters();
        shooterSub.getInterpLUT();

        //telemetryMethods.getRobotTelemetry();


        if (A1 == 1) {
            mecanumDriveSub.aprilTagTracking(LSx1, LSy1);
        } else {
            mecanumDriveSub.driveRobotPOV(LSx1, LSy1, RSx1);
        }

        if (B2 == 1) {
            intakeFeederSub.stop();
        } else {
            intakeFeederSub.intake(-LSy2);
            intakeFeederSub.feeder(-RSy2);
        }
        if (B2 == 1) {
            shooterSub.stop();
        } else if (Y2 == 1) {
            timer.reset();
            subsystemsInitializer.automatizedShoot();
            shooterSub.getGetters();
            if  (shooterSub.getShooterVel() >= shooterSub.desiredRPMs){
                time = timer.seconds();
            }
        } else {
            once = false;
            onceSaved = false;
            shooterSub.shootManually(LT2 > .05 ? -LT2 : RT2);

        }


        //subsystemsInitializer.getCompensationTimer();
        //dashboard.getStabilizationTime();

        dashboard.printpene(time);
    }

    @Override
    public void stop() {
        subsystemsInitializer.absStop();
    }

}
