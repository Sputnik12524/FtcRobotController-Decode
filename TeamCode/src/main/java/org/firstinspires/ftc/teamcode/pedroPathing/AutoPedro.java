package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoPedro extends LinearOpMode {
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
        follower.setStartingPose(new Pose(22, 124, Math.toRadians(-38)));

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
        public PathChain PathFirstScoring, PathToPresetArtifacts, PathIntakingArtifacts,
                PathSecondScoring, PathLeaving;

        public Paths(Follower follower) {
            PathFirstScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 124.000),

                                    new Pose(43.541, 103.509)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.541, 103.509),

                                    new Pose(43.258, 84.561)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-38), Math.toRadians(180))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.258, 84.561),

                                    new Pose(16.940, 83.897)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.940, 83.897),

                                    new Pose(43.777, 103.708)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-38))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.777, 103.708),

                                    new Pose(58.119, 132.225)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.shootByVelocity();
                in.rotateIn();
                setPathState(1);
                break;
            case 1:
                if (sh.getVelocityRPS() >= Shooter.VELOCITY_FOR_LONG_THROW) {
                    follower.followPath(Pa);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
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
