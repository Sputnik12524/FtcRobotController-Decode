package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue;

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
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "BLUE 18 short", group = "Autonomous")
public class Auto18ArtefactsShortBlue extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Logger lg;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    AutoSniper as;
    Turret tt;
    Limelight ll;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        in = new Intake(this);
        ll = new Limelight(this);
        lg = new Logger("pospos");
        tt = new Turret(this, ll);
        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 127, Math.toRadians(-36)));
        sh = new Shooter(this, follower);
        as = new AutoSniper(tt, sh, ll);

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        ll.startOrStopLL(false);

        waitForStart();
        while (opModeIsActive()) {
            ll.update();
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose

            tt.turnByTarget(-17);

            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Turret ", tt.getCurrentPosOfTurret());
            // t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
        tt.turnByTarget(0);
        tt.turretRegulator.interrupt();
        ll.startOrStopLL(true);
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathSecondPresentArtefacts; //48.60
        public final PathChain PathSecondIntakingArtefacts;  //17.60
        public final PathChain PathThirdScoring; // 47.115
        public final PathChain PathTGatePresentArtefacts;
        public final PathChain PathGateIntakingArtefacts;
        public final PathChain PathGateScoring;
        public final Pose forGateNym = new Pose(25, 67);
        public final Pose gateNym = new Pose(15, 62);

        public final Pose scoringPose = new Pose(43, 120);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22, 127),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-180))

                    .build();

            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(

                                    scoringPose,

                                    new Pose(48, 84) //55,100
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48, 84), //55,100

                                    new Pose(20, 84) //35,100
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15, 71), //35,100

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(-180)) //-36

                    .build();

            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(50, 56)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50, 56),

                                    new Pose(23, 50)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();


            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23, 50),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathTGatePresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    forGateNym
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(154))

                    .build();

            PathGateIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    forGateNym,

                                    gateNym
                            )

                    ).setConstantHeadingInterpolation(Math.toRadians(154))
                    .build();


            PathGateScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateNym,

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(154))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(42, 130)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.closeTunnel();
                sh.shootByVelocity();
                in.rotateIn();
                follower.followPath(paths.PathScoring);
                setPathState(1);
                break;

            case 1:
                if (!sh.isSpinUp() || follower.isBusy()) break;
                sh.openTunnel();
                setPathState(2);
                break;

            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1100) break;
                follower.followPath(paths.PathToPresetArtifacts);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    sh.closeTunnel();
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() || actionTimer.getElapsedTime() > 500) {
                    in.rotateIn();
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!sh.isSpinUp() || follower.isBusy()) break;
                sh.openTunnel();
                setPathState(8);
                break;

            case 8:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                follower.followPath(paths.PathSecondPresentArtefacts, true);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    sh.closeTunnel();
                    in.rotateIn();
                    follower.followPath(paths.PathSecondIntakingArtefacts, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathThirdScoring, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!sh.isSpinUp() || follower.isBusy()) break;
                sh.openTunnel();
                setPathState(12);
                break;

            case 12:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                follower.followPath(paths.PathTGatePresentArtefacts);
                setPathState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    sh.closeTunnel();
                    in.rotateIn();
                    follower.followPath(paths.PathGateIntakingArtefacts);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.PathGateScoring);
                    setPathState(16);
                }
                break;

            case 16:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(17);
                }
                break;

            case 17:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                follower.followPath(paths.PathTGatePresentArtefacts);
                setPathState(18);
                break;

            case 18:
                if (!follower.isBusy()) {
                    sh.closeTunnel();
                    in.rotateIn();
                    follower.followPath(paths.PathGateIntakingArtefacts);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.PathGateScoring);
                    setPathState(20);
                }
                break;

            case 20:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(21);
                }
                break;

            case 21:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                follower.followPath(paths.PathTGatePresentArtefacts);
                setPathState(22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    sh.closeTunnel();
                    in.rotateIn();
                    follower.followPath(paths.PathGateIntakingArtefacts);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.PathGateScoring);
                    setPathState(24);
                }
                break;

            case 24:
                if (sh.isSpinUp() && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(25);
                }
                break;

            case 25:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1100) break;
                tt.turnByTarget(0);
                in.rotateStop();
                sh.shootStop();
                follower.followPath(paths.PathLeaving);
                setPathState(-101);
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}
