package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red.far;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;

@Autonomous(name = "9  Long  RED", group = "Autonomous")
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
    Logger lg;
    Turret tt;

    @Override
    public void runOpMode() {
        actionTimer = new Timer();
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(90, 8, Math.toRadians(-90)));

        in = new Intake(this);
        Limelight ll = new Limelight(this);
        tt = new Turret(this, ll);
        sh = new Shooter(this);
        lg = new Logger("pospos");


        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.closeTunnel();
        sh.setLongThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
        tt.turretRegulator.start();

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
        tt.turretRegulator.interrupt();
    }


    public static class Paths {

        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring, PathThirdPreset, PathThirdScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 8),

                                    new Pose(83, 36)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 36),

                                    new Pose(125, 36)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 36),

                                    new Pose(90, 8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))

                    .build();


            PathThirdPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 12),

                                    new Pose(134, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();
            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134, 10),
                                    new Pose(90, 12)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 12),
                                    new Pose(83, 58)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.shootByVelocity();
                in.rotateIn();
                setPathState(1);
                break;
            case 1:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 4000) {
                    sh.closeTunnel();
                    follower.followPath(paths.PathToPresetArtifacts);
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
                    setPathState(5);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                sh.closeTunnel();
                follower.followPath(paths.PathThirdPreset);
                setPathState(8);
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.PathThirdScoring);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    sh.openTunnel();
                    setPathState(10);
                }
                break;
            case 10:
                if(follower.isBusy() || actionTimer.getElapsedTime()<4000) break;
                tt.turnByTarget(0);
                follower.followPath(paths.PathLeaving);
                in.rotateStop();
                sh.shootStop();
                lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                lg.fileClose();
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
