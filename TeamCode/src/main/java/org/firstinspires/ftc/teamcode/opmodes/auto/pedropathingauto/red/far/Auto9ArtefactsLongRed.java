package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red.far;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;

@Autonomous(name = "9  Long  RED", group = "Autonomous")
@Disabled
@Configurable // Panels
public class Auto9ArtefactsLongRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        in = new Intake(this);
        sh = new Shooter(this);

        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(90, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.closeTunnel();
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
        public final PathChain PathSecondScoring, PathThirdPreset, PathThirdIntaking, PathThirdScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathFirstScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 8),

                                    new Pose(83, 18)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(62))

                    .build();

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 18),

                                    new Pose(93, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(0))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93, 35),

                                    new Pose(125, 35)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 35),

                                    new Pose(83, 18)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(62))

                    .build();

            PathThirdPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 18),

                                    new Pose(93, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(0))

                    .build();
            PathThirdIntaking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93, 10),
                                    new Pose(125, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 10),
                                    new Pose(83, 18)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(62))
                    .build();
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 18),
                                    new Pose(83, 58)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(62))
                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.shootByVelocity();
                in.rotateIn();
                follower.followPath(paths.PathFirstScoring, true);
                setPathState(1);
                break;
            case 1:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                sh.closeTunnel();
                follower.followPath(paths.PathToPresetArtifacts, true);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                sh.closeTunnel();
                follower.followPath(paths.PathThirdPreset);
                setPathState(9);

                break;
            case 9:
                if(!follower.isBusy()) {
                    in.rotateIn();
                    follower.followPath(paths.PathThirdIntaking);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(paths.PathThirdScoring);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    sh.openTunnel();
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.isBusy() || actionTimer.getElapsedTime()<4000) break;
                follower.followPath(paths.PathLeaving);
                in.rotateStop();
                sh.shootStop();
                setPathState(-100);
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}
