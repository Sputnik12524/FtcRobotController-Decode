package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "RED 6 short", group = "Autonomous")
@Configurable // Panels
public class Auto6ArtifactsShortRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    ElapsedTime loggerTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Logger lg;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        actionTimer = new Timer();
        actionTimer.resetTimer();
        loggerTimer = new ElapsedTime();
        opmodeTimer.resetTimer();

        in = new Intake(this);
        sh = new Shooter(this);
        lg = new Logger("pospos");

        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 122, Math.toRadians(-136)));

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

            if (loggerTimer.milliseconds() > 750) {
                lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), 0);
                loggerTimer.reset();
            }
        }
        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), 0);
        lg.fileClose();
    }


    public static class Paths {
        public final PathChain PathFirstScoring;
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathLeaving;
        public final Pose scoringPose = new Pose(100, 111);

        public Paths(Follower follower) {
            PathFirstScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136)) //was tangent

                    .build();
            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(100, 84) // 93,95
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 84),

                                    new Pose(125, 84) // 106, 95
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 84),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(85, 127)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(36))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.shootByVelocity();
                in.rotateIn();
                follower.followPath(paths.PathFirstScoring, true);
                setPathState(1);
                break;
            case 1:
                if (!sh.isSpinUp() || follower.isBusy()) break;
                sh.openTunnel();
                setPathState(3);
                break;
            case 3:
                if (actionTimer.getElapsedTime() >= 4000 && !follower.isBusy()) {
                    sh.closeTunnel();
                    follower.followPath(paths.PathToPresetArtifacts, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    sh.openTunnel();
                    //   sh.waitForShoot();
                    setPathState(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTime() >= 4000 && !follower.isBusy()) {
                    sh.shootStop();
                    in.rotateStop();
                    follower.followPath(paths.PathLeaving);

                    setPathState(-1);
                }
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}
