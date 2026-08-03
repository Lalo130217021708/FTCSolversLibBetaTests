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

@Autonomous(name = "Red Using Gate", group = "Tests")
public class RedUsingGate extends LinearOpMode {
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    public PathChain MainChain;
    public PathChain Chain2;
    public PathChain Chain3;
    public PathChain Chain4;
    public PathChain Chain5;
    public PathChain Chain6;
    public PathChain Chain7;


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
                                new Pose(119.000, 123.100),
                                new Pose(96.499, 94.863)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(44))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.499, 94.863),
                                new Pose(97.127, 60.729)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97.127, 60.729),
                                new Pose(123.628, 60.666)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(123.628, 60.666),
                                new Pose(117.865, 60.675)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(117.865, 60.675),
                                new Pose(90.312, 72.558),
                                new Pose(83.158, 82.540)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83.158, 82.540),
                                new Pose(122.540, 82.144)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.540, 82.144),
                                new Pose(96.262, 94.622)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmd,
                follow(follower, Chain2),
                race(follow(follower, Chain3),intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain4),
                subsystemsInitializer.automatizedShootCmd,
                follow(follower, Chain5),
                race(follow(follower, Chain6), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain7),
                subsystemsInitializer.automatizedShootCmd,
                instant(this::terminateOpModeNow)
        );
    }
}
