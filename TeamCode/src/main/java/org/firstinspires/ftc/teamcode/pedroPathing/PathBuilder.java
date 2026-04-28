package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PathBuilder {
    private Follower follower;

    public PathBuilder(Follower follower) {
        this.follower = follower;
    }

    public void buildPath(PathChain path, Pose startPose, Pose endPose){
        //noinspection ReassignedVariable
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void buildPathHeading(PathChain path, Pose startPose, Pose endPose){
        //noinspection ReassignedVariable
        path = follower.pathBuilder()
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }
}
