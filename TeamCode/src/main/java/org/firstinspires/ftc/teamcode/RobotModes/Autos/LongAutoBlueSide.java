package org.firstinspires.ftc.teamcode.RobotModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;

import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Long Auto Blue Side", group = "Tests")
public class LongAutoBlueSide extends LinearOpMode {
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    Pose startPose = new Pose(72, 8, Math.toRadians(90));
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
    public PathChain Chain5;
    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        subsystemsInitializer = new SubsystemsInitializer(hardwareMap);
        telemetryMethods = new TelemetryMethods(telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        schedule(routine());
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void buildPaths(){

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(59.000, 8.500),
                                new Pose(61.204, 12.776)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.204, 12.776),
                                new Pose(55.292, 35.257)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(53.292, 35.257),
                                new Pose(21.430, 35.401)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(21.430, 35.401),
                                new Pose(52.900, 27.980),
                                new Pose(62.837, 10.664)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(65.837, 10.664),
                                new Pose(45.844, 20.205)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public Command routine(){
        return sequential(
                /// Start to Shooting Pose ///
                follow(follower, Chain2),
                /// Shooting Time !!! ///
                subsystemsInitializer.automatizedShootCmdLong,
                /// Shooting Pose to Intaking second row ///
                follow(follower, Chain3),
                /// Second Intake Pose to open Gate ///
                race(follow(follower, Chain4), intakeFeederSub.intakeFeederCdm),
                /// Gate to shooting Pose ///
                subsystemsInitializer.automatizedShootCmdLong,
                follow(follower, Chain5),
                /// From Intake to shooting pose ///
                instant(this::terminateOpModeNow)
        );
    }
}

