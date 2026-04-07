package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    Follower follower;

    public Paths(Follower follower) {
        this.follower = follower;
    }

    public PathChain blueParking(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                new Pose(105.5, 33)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

    }

    public PathChain redParking(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                new Pose(38.5, 33)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain blueHuman(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                new Pose(93, 8)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain redHuman(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                               start,
                                new Pose(50, 8)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain blueGoal(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                new Pose(42, 105)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public PathChain redGoal(Pose start) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                new Pose(95, 105)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }
}
