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
import static org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer.shooterSub;

import org.firstinspires.ftc.teamcode.Initializers.SubsystemsInitializer;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeFeederSub;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "First Real Auto", group = "Tests")
public class FirstRealAuto extends LinearOpMode {
    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;
    Pose startPose = new Pose(21,121,Math.toRadians(143.5));
    Pose shootingFtPose = new Pose(60.819, 78, Math.toRadians(135));
    Pose firstIntakePose = new Pose(14, 78, Math.toRadians(180));
    Pose shootingScPose = new Pose(37.382, 103.629, Math.toRadians(137.5));
    Pose secondIntakePose = new Pose(44.042, 53.5, Math.toRadians(180));
    Pose finishScdIntakePose = new Pose(13.5, 53.5, Math.toRadians(180));
    Pose controlPointPose = new Pose(39.189, 64.825, Math.toRadians(143.5));
    Pose shootingThrdPose = new Pose(48.361, 92.467, Math.toRadians(133));
    Pose lastPose = new Pose(19.193,59.608, Math.toRadians(143.5));
    PathChain firstLine, secondLine, thirdLine, fourthLine, fifthLine, sixthLine, seventhLine, eighthLine;


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
        firstLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingFtPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingFtPose.getHeading())
                .build();
        thirdLine = follower.pathBuilder()
                .addPath(new BezierLine(shootingFtPose, firstIntakePose))
                .setConstantHeadingInterpolation(firstIntakePose.getHeading())
                .build();
        fourthLine = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose, shootingScPose))
                .setLinearHeadingInterpolation(firstIntakePose.getHeading(), shootingScPose.getHeading())
                .build();
        fifthLine = follower.pathBuilder()
                .addPath(new BezierLine(shootingScPose, secondIntakePose))
                .setLinearHeadingInterpolation(shootingScPose.getHeading(), secondIntakePose.getHeading())
                .build();
        sixthLine = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose, finishScdIntakePose))
                .setLinearHeadingInterpolation(secondIntakePose.getHeading(), finishScdIntakePose.getHeading())
                .build();
        seventhLine = follower.pathBuilder()
                .addPath(new BezierCurve(finishScdIntakePose, controlPointPose, shootingThrdPose))
                .setLinearHeadingInterpolation(finishScdIntakePose.getHeading(), shootingThrdPose.getHeading())
                .build();
        eighthLine = follower.pathBuilder()
                .addPath(new BezierLine(shootingThrdPose, lastPose))
                .setLinearHeadingInterpolation(shootingThrdPose.getHeading(), lastPose.getHeading())
                .build();
    }
    public Command routine(){
        return sequential(
                follow(follower, firstLine),
                subsystemsInitializer.automatizedShootCmd,
                race(follow(follower, thirdLine), intakeFeederSub.intakeFeederCdm),
                follow(follower, fourthLine),
                subsystemsInitializer.automatizedShootCmd,
                follow(follower, fifthLine),
                race(follow(follower, sixthLine), intakeFeederSub.intakeFeederCdm),
                follow(follower, seventhLine),
                subsystemsInitializer.automatizedShootCmd,
//                follow(follower, eighthLine),
                instant(this::terminateOpModeNow)
        );
    }
}
