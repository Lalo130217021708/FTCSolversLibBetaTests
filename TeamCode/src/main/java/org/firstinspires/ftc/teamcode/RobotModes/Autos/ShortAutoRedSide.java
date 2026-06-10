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

@Autonomous(name = "Short Red Side", group = "Tests")
public class ShortAutoRedSide extends LinearOpMode {
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
                                new Pose(92.180, 90.518)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(50))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(92.180, 90.518),
                                new Pose(99, 58.427)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(99, 58.427),
                                new Pose(133.148, 57.813)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.148, 57.813),
                                new Pose(98.088, 61.400),
                                new Pose(129.673, 63.677)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.673, 63.677),
                                new Pose(103.594, 64.753),
                                new Pose(82.931, 82.219)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47.5))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.931, 82.219),
                                new Pose(103.015, 82.108)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(47.5), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(103.015, 82.108),
                                new Pose(125.638, 81.971)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.638, 81.971),
                                new Pose(106.311, 103.585)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47.5))
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, Chain2), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain3),
                follow(follower, Chain4),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, Chain5), intakeFeederSub.intakeFeederCdm),
                follow(follower, Chain6),
                subsystemsInitializer.automatizedShootCmd,
                instant(this::terminateOpModeNow)
        );
    }
}
