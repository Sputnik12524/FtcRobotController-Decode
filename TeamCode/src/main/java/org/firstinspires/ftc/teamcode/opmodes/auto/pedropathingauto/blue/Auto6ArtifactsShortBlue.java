package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue;

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
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BLUE 6 Short", group = "Autonomous")
@Configurable // Panels
public class Auto6ArtifactsShortBlue extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Logger lg;
    private Timer actionTimer;

    Turret tt;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Limelight ll;
    AutoSniper as;
    ElapsedTime loggerTimer;


    @Override
    public void runOpMode() {
        lg = new Logger("pospos");
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();
        actionTimer.resetTimer();
        loggerTimer = new ElapsedTime();

        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 127, Math.toRadians(-36)));


        in = new Intake(this);
        sh = new Shooter(this, follower);
        ll = new Limelight(this);
        tt = new Turret(this,ll);
        as = new AutoSniper(tt, sh);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.openTunnel();
        sh.setShortThrowMode();
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();

        waitForStart();
        while (opModeIsActive()) {
            ll.update();
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose

            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
            if (loggerTimer.milliseconds() > 750) {
                lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
                loggerTimer.reset();
            }
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
        lg.fileClose();
        tt.turretRegulator.interrupt();
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
                                    new Pose(22, 127),

                                    new Pose(47, 100)))

                    .setConstantHeadingInterpolation(Math.toRadians(-36))

                    .build();

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 100),

                                    new Pose(44, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-180))

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

                                    new Pose(47, 100)))

                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-36))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 100),

                                    new Pose(58, 132)))
                    .setConstantHeadingInterpolation(Math.toRadians(-36))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            sh.closeTunnel();
            sh.shootByVelocity();
            in.rotateIn();
            follower.followPath(paths.PathFirstScoring);
            setPathState(1);
            break;
            case 1:
                if (!sh.isSpinUp()||follower.isBusy()) break;
                sh.openTunnel();
                setPathState(2);
                break;
            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1500) break;
                follower.followPath(paths.PathToPresetArtifacts);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    sh.closeTunnel();
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
                if (!sh.isSpinUp()||follower.isBusy()) break;
                sh.openTunnel();
                setPathState(7);

                break;
            case 7:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1100) break;
                as.enableAutoTurretAiming(false);
                in.rotateStop();
                sh.shootStop();
                follower.followPath(paths.PathLeaving);
                setPathState(-100);

                break;

        }

    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
