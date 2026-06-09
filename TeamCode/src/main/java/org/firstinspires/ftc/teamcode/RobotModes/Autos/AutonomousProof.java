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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Squirt", group = "Tests")
public class AutonomousProof extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(30,25,Math.toRadians(90));
    private PathChain MainChain, Chain2, Chain3, Chain4;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
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
                                new Pose(30.000, 25.000),
                                new Pose(30.000, 78.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(30.000, 78.000),
                                new Pose(110.000, 78.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Chain3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(110.000, 78.000),
                                new Pose(110.000, 25.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        Chain4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(110.000, 25.000),
                                new Pose(30.000, 25.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    public Command routine(){
        return sequential(
                follow(follower, MainChain),
                follow(follower, Chain2),
                follow(follower, Chain3),
                follow(follower, Chain4),
                instant(this::terminateOpModeNow)
        );
    }
}
