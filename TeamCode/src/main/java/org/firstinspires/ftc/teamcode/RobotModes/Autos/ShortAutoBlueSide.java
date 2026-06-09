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

import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Short Auto Blue Side", group = "Tests")
public class ShortAutoBlueSide extends LinearOpMode {
    Follower follower;
    IntakeFeederSub intakeFeederSub;
    Pose startPose = new Pose(21,121,Math.toRadians(143.5));
    PathChain MainChain, Chain2, Chain3, Chain4, Chain5, Chain6, Chain7, Chain8;
    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        intakeFeederSub = new IntakeFeederSub(hardwareMap);
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
                                startPose,
                                new Pose(49.210, 92.074)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(143.5))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.210, 92.074),
                                new Pose(49.554, 58.703)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.554, 58.703),
                                new Pose(10.295, 58.046)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.295, 58.046),
                                new Pose(57.170, 65.055),
                                new Pose(48.729, 93.607)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();

        Chain5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.729, 93.607),
                                new Pose(43.177, 63.009),
                                new Pose(9.892, 59.001)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(143.5))
                .build();

        Chain6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.892, 59.001),
                                new Pose(41.350, 50.938),
                                new Pose(60.674, 82.578)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(133))
                .build();

        Chain7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.674, 82.578),
                                new Pose(14.676, 82.578)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Chain8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.676, 82.578),
                                new Pose(43.972, 96.831)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
    }

    public Command routine(){
        return sequential(
                /// Start to Shooting Pose ///
                follow(follower, MainChain),
                /// Shooting Pose to initial second Intake Pose ///
                follow(follower, Chain2),
                /// Intaking second row ///
                race(follow(follower, Chain3), intakeFeederSub.intakeFeederCdm),
                /// Second Intake Pose to shooting Pose ///
                follow(follower, Chain4),
                /// Shooting Pose to Gate ///
                follow(follower, Chain5),
                /// Gate to shooting Pose ///
                follow(follower, Chain6),
                /// Shooting pose to intaking first Row ///
                follow(follower, Chain7),
                /// From Intake to shooting pose ///
                follow(follower, Chain8),
                instant(this::terminateOpModeNow)
        );
    }
}
