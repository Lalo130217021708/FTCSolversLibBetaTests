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
    private final Pose startPose = new Pose(0,0,Math.toRadians(90));
    private final Pose topLPose = new Pose(40,0,Math.toRadians(90));
    private final Pose topRPose = new Pose(40,40,Math.toRadians(90));
    private final Pose rearRightPose = new Pose(0,40,Math.toRadians(90));
    private final Pose endPose = new Pose(0,0,Math.toRadians(90));
    private PathChain firstLine, secondLine, thirdLine, fourthLine;

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
        firstLine = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, topLPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), topLPose.getHeading())
                    .build();
        secondLine = follower.pathBuilder()
                .addPath(new BezierLine(topLPose, topRPose))
                .setLinearHeadingInterpolation(topLPose.getHeading(), topRPose.getHeading())
                .build();
        thirdLine = follower.pathBuilder()
                .addPath(new BezierLine(topRPose, rearRightPose))
                .setLinearHeadingInterpolation(topRPose.getHeading(), rearRightPose.getHeading())
                .build();
        fourthLine = follower.pathBuilder()
                .addPath(new BezierLine(rearRightPose, endPose))
                .setLinearHeadingInterpolation(rearRightPose.getHeading(), endPose.getHeading())
                .build();
    }

    public Command routine(){
        return sequential(
                follow(follower, firstLine),
                follow(follower, secondLine),
                follow(follower, thirdLine),
                follow(follower, fourthLine),
                instant(this::terminateOpModeNow)
        );
    }
}
