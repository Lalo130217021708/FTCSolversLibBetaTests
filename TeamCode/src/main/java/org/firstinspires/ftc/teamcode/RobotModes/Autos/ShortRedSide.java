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

/// THIS IS FOR BLUE SIDE
@Autonomous(name = "Short Red Side", group = "Tests")
public class ShortRedSide extends LinearOpMode {
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
        follower.setStartingPose(new Pose(118.8, 123.7, Math.toRadians(37.5)));

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
                                new Pose(118.800, 123.700),
                                new Pose(87.392, 81.326)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(48))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.392, 81.326),
                                new Pose(95.754, 81.654)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(95.754, 81.654),
                                new Pose(121.267, 81.860)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(121.267, 81.860),
                                new Pose(101.411, 87.430),
                                new Pose(99.034, 99.198)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(99.034, 99.198),
                                new Pose(82.741, 76.710),
                                new Pose(96.617, 58.389)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(96.617, 58.389),
                                new Pose(120.933, 58.095)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.933, 58.095),
                                new Pose(96.821, 69.396),
                                new Pose(97.565, 96.766)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmd,
                /// intaking
                race(follow(follower, Chain2), intakeFeederSub.intakeFeederCdm),
                /// shooting
                follow(follower, Chain3),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, Chain4), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain6),
                subsystemsInitializer.automatizedShootCmd,
                instant(this::terminateOpModeNow)
        );
    }
}
