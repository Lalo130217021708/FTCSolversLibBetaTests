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

/// THIS IS FOR RED SIDE
@Autonomous(name = "Blue Using Gate", group = "Tests")
public class BlueUsingGate extends LinearOpMode {
    public PathChain MainChain;
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
    public PathChain Chain5;
    public PathChain Chain6;
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    Pose startPose = new Pose(22.6,123.7,Math.toRadians(144));

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
    public void buildPaths() {
        MainChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.600, 123.700),
                                new Pose(55.016, 86.009)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(147.5), Math.toRadians(135))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.016, 86.009),
                                new Pose(47.243, 60.947)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(47.243, 60.947),
                                new Pose(20.738, 60.022)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.738, 60.022),
                                new Pose(54.229, 69.377),
                                new Pose(56.370, 83.586)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.370, 83.586),
                                new Pose(21.549, 82.520)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.549, 82.520),
                                new Pose(38.626, 101.709)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmd,
                follow(follower,Chain2),
                /// intaking
                race(follow(follower, Chain3), intakeFeederSub.intakeFeederCdm),
                /// shooting
                follow(follower,Chain4),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, Chain5), intakeFeederSub.intakeFeederCdm),
                follow(follower,Chain6),
                subsystemsInitializer.automatizedShootCmd,
                instant(this::terminateOpModeNow)
        );
    }
}
