package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue.far;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
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

@Autonomous(name = " BLUE 6 Long", group = "Autonomous")
@Configurable // Panels
public class Auto6ArtifactsLongBlue extends LinearOpMode {
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
    AutoSniper as;

    @Override
    public void runOpMode() {
        actionTimer = new Timer();
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(-90)));


        Limelight ll = new Limelight(this);
        in = new Intake(this);
       tt = new Turret(this,ll);
        sh = new Shooter(this, follower, new Transfer(this));
        lg = new Logger("pospos");
        as = new AutoSniper(tt,sh);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.closeTunnel();
        sh.setLongThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();


        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(),follower.getHeading());
            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.debug("Velocity", sh.getVelocityRPS());
            panelsTelemetry.update(telemetry);
        }

        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
        tt.turretRegulator.interrupt();
    }


    public static class Paths {

        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56, 8),

                                    new Pose(41, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41, 35),

                                    new Pose(20, 35)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 35),

                                    new Pose(56, 8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56, 8),

                                    new Pose(45, 35)
                            )
                    ).setConstantHeadingInterpolation(-90)

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.shootByVelocity();
                setPathState(1);
                break;
            case 1:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    in.rotateIn();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 2500) {
                    as.enableAutoTurretAiming(false);
                    in.rotateStop();
                    sh.closeTunnel();
                    follower.followPath(paths.PathToPresetArtifacts);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && actionTimer.getElapsedTime() >= 1200) {
                    as.enableAutoTurretAiming(true);
                    in.rotateIn();
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    // in.rotateStop();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    in.rotateIn();
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() && actionTimer.getElapsedTime()>3000){
                    as.enableAutoTurretAiming(false);
                    in.rotateStop();
                    sh.shootStop();
                    follower.followPath(paths.PathLeaving);
                    setPathState(-100);
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
