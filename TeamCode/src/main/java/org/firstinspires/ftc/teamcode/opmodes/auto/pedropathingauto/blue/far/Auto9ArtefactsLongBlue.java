package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "BLUE 9 Long", group = "Autonomous")
public class Auto9ArtefactsLongBlue extends LinearOpMode {
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
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();
        Transfer tr = new Transfer(this);

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(-90)));

        in = new Intake(this);
        Limelight ll = new Limelight(this);
        tt = new Turret(this, ll);
        sh = new Shooter(this);
        lg = new Logger("pospos");
        as = new AutoSniper(tt, sh);

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setLongThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW + 0.175);
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        tt.turretRegulator.interrupt();
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
    }


    public static class Paths {

        public final PathChain PathLeaving;
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathSecondPresetArtefacts; //48.60
       // public final PathChain PathSecondIntakingArtefacts;  //17.60
        public final PathChain PathThirdScoring; // 47.115
        public final Pose scoringPose = new Pose(56, 8);


        public Paths(Follower follower) {

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(41, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-180))

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

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();
//
//            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    scoringPose,
//
//                                    new Pose(47, 65)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(-180))
//
//                    .build();


            //   Если берем из зоны хьюмана


            PathSecondPresetArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(10, 8)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();


//            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(40, 8),
//
//                                    new Pose(10, 10)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-165))
//
//                    .build();

            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 8),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

//            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(47, 70),
//
//                                    new Pose(20, 70)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(-180))
//
//                    .build();

//            PathThirdScoring = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(20, 70),
//
//                                    scoringPose
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(110))
//
//                    .build();


            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(45, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))

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
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1500) {
                    // as.enableAutoTurretAiming(false);
                    in.rotateStop();
                    sh.closeTunnel();
                    follower.followPath(paths.PathToPresetArtifacts);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && actionTimer.getElapsedTime() >= 1000) {
                    // as.enableAutoTurretAiming(true);
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
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    in.rotateStop();
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    in.rotateIn();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1500) {
                    sh.closeTunnel();
                    follower.followPath(paths.PathSecondPresetArtefacts, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && actionTimer.getElapsedTime() >= 1000) {
                    // as.enableAutoTurretAiming(true);
                    in.rotateIn();
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathThirdScoring);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    in.rotateStop();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    in.rotateIn();
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 2500) {
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
