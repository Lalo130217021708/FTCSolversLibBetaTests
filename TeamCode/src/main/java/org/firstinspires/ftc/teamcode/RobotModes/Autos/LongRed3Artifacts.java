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

@Autonomous
        (name = "Long Red 3 Artifacts", group = "tests")
public class LongRed3Artifacts extends LinearOpMode {

    Follower follower;
    SubsystemsInitializer subsystemsInitializer;
    TelemetryMethods telemetryMethods;

    Pose startPose = new Pose(89.94032444959443, 7.999999999999984, Math.toRadians(90));

    PathChain MainChain;
    PathChain Chain2;


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
                                new Pose(89.940, 8.000),
                                new Pose(82.070, 15.997)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63))
                .build();

        Chain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.070, 15.997),
                                new Pose(106.157, 10.062)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(90))
                .build();
    }

    public Command routine(){
        return sequential(
                /// Start to Shooting Pose ///
                follow(follower, MainChain),
                /// Shooting Time !!! ///
                subsystemsInitializer.automatizedShootCmdLong,
                follow(follower,Chain2),
                instant(this::terminateOpModeNow)
        );
    }
}
