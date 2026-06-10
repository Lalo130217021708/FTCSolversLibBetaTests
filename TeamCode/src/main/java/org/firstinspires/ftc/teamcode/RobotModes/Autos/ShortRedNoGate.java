package org.firstinspires.ftc.teamcode.RobotModes.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.intakeFeederSub;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Short Red No Gate", group = "Tests")
public class ShortRedNoGate extends LinearOpMode {
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    public PathChain MainChain;
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
    public PathChain Chain5;
    public PathChain Chain6;


    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        subsystemsInitializer = new SubsystemsInitializer(hardwareMap);
        telemetryMethods = new TelemetryMethods(telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(119, 123, Math.toRadians(37.5)));

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
                                new Pose(119.000, 123.000),
                                new Pose(84.182, 82.798)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(43))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.182, 82.798),
                                new Pose(100.450, 82.151)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(100.450, 82.151),
                                new Pose(126.899, 81.598)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.899, 81.598),
                                new Pose(99.819, 99.460)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(99.819, 99.460),
                                new Pose(98.586, 58.219)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(98.586, 58.219),
                                new Pose(126.753, 57.919)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.753, 57.919),
                                new Pose(86.489, 86.592)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, Chain2), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain3),
                subsystemsInitializer.automatizedShootCmd,
                follow(follower, Chain4),
                race(follow(follower, Chain5), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain6),
                subsystemsInitializer.automatizedShootCmd,
                instant(this::terminateOpModeNow)
        );
    }
}
