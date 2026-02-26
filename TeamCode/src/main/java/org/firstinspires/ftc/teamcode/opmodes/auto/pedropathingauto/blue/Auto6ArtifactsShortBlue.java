package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;

@Autonomous(name = "Auto 6 Short BLUE", group = "Autonomous")
@Configurable // Panels
public class Auto6ArtifactsShortBlue extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Logger lg;
    private Timer actionTimer;
    //private Turret tt;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;

    @Override
    public void runOpMode() {
        lg = new Logger("pospos");
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        in = new Intake(this);
        sh = new Shooter(this);
        //tt = new Turret(this);

        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 124, Math.toRadians(-38)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.openTunnel();
        sh.setShortThrowMode();

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose


            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

    }


    public static class Paths {
        public final PathChain PathFirstScoring;
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathFirstScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22, 124),

                                    new Pose(44, 104)))

                    .setConstantHeadingInterpolation(Math.toRadians(-40))

                    .build();

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44, 104),

                                    new Pose(44, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(-38), Math.toRadians(-180))

                    .build();

            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44, 85),

                                    new Pose(30, 85)))
                    .setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30, 85),

                                    new Pose(44, 104)))

                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-40))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44, 104),

                                    new Pose(58, 132)))
                    .setConstantHeadingInterpolation(Math.toRadians(-40))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.openTunnel();
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.shootByVelocity();
                setPathState(1);
                break;

            case 1:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    follower.followPath(paths.PathFirstScoring, true);
                    in.rotateIn();
                    setPathState(2);
                }

                break;

            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 4000) {
                    sh.closeTunnel();
                    in.rotateStop();
                    follower.followPath(paths.PathToPresetArtifacts, true);
                    in.rotateIn();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!sh.isSpinUp()) {
                    in.rotateIn();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathLeaving);
                    sh.closeTunnel();
                    lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    lg.fileClose();
                    setPathState(-100);
                }
                    break;

        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
