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
    Pose startPose = new Pose(51.68331402085747, 8.16396292004632, Math.toRadians(90));
    public PathChain MainChain;
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
    public PathChain Chain5;

    public PathChain Chain6;
    @Override
    public void runOpMode() throws InterruptedException{
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
        MainChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(51.683, 8.000),
                                new Pose(57.642, 15.431)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.642, 15.431),
                                new Pose(60.925, 28.869),
                                new Pose(49.441, 34.688)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(49.441, 34.688),
                                new Pose(13.457, 35.111)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.457, 35.111),
                                new Pose(57.794, 12.549)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.794, 12.549),
                                new Pose(46.514, 59.247)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(46.514, 59.247),
                                new Pose(24.560, 59.596)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24.560, 59.596),
                                new Pose(63.562, 77.826)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5))
                .build();
    }

    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmdLong,
                /// Start to Shooting Pose ///
                race(follow(follower, Chain2), intakeFeederSub.intakeFeederCdm),
                follow(follower,Chain3),
                /// Shooting Time !!! ///
                subsystemsInitializer.automatizedShootCmdLong,
                /// Shooting Pose to Intaking second row ///
//                follow(follower, Chain4),
//                /// Second Intake Pose to open Gate ///
//                race(follow(follower, Chain5), intakeFeederSub.intakeFeederCdm),
//                follow(follower,Chain6),
//                /// Gate to shooting Pose ///
//                subsystemsInitializer.automatizedShootCmdLong,
//                /// From Intake to shooting pose ///
                instant(this::terminateOpModeNow)
        );
    }
}

