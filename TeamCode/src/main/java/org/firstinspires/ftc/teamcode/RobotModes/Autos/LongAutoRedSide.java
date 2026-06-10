package org.firstinspires.ftc.teamcode.RobotModes.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Long Red Side", group = "Tests")
public class LongAutoRedSide extends LinearOpMode {
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    Pose startPose = new Pose(72, 8, Math.toRadians(90));
    public PathChain MainChain;
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
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
        MainChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(80.539, 8.500),
                                new Pose(81.530, 12.560)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72.5))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(81.530, 12.560),
                                new Pose(95, 38.694)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(72.5), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(95, 37.694),
                                new Pose(126.790, 37.837)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.790, 38.837),
                                new Pose(79.343, 12.744)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(72.5))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(79.343, 12.744),
                                new Pose(79.299, 33.950)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(72.5), Math.toRadians(0))
                .build();
    }

    public Command routine(){
        return sequential(
                /// Start to Shooting Pose ///
                follow(follower, MainChain),
                /// Shooting Time !!! ///
                subsystemsInitializer.automatizedShootCmdLong,
                /// Shooting Pose to Intaking second row ///
                /// Second Intake Pose to open Gate ///
                race(follow(follower, Chain2), intakeFeederSub.intakeFeederCdm),
                /// Gate to shooting Pose ///
                follow(follower, Chain3),
                /// Shooting Time !!! ///
                subsystemsInitializer.automatizedShootCmdLong,
                /// Leave the shooting zone !!!
                follow(follower, Chain4),
                /// From Intake to shooting pose ///
                instant(this::terminateOpModeNow)
        );
    }
}

