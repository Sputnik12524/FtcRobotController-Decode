package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "RED PlayOff", group = "Autonomous")
public class Auto9ArtefactsShortPlayOffRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    ElapsedTime timer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    AutoSniper as;
    Shooter sh;
    Logger lg;
    Transfer tr;
    Turret tt;
    public static double TURRET_WAIT = 3500;


    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        timer = new ElapsedTime();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 122, Math.toRadians(-136)));

        tr = new Transfer(this);

        in = new Intake(this);
        Limelight ll = new Limelight(this);
        tt = new Turret(this, ll);
        sh = new Shooter(this, follower);
        lg = new Logger("pospos");
        as = new AutoSniper(tt, sh, ll);
        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        as.setAlliance(Alliance.RED);
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
        tt.turretRegulator.start();
        ll.startOrStopLL(false);


        waitForStart();
        while (opModeIsActive()) {
            ll.update();
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
            telemetry.addLine(String.valueOf((int) (follower.getPose().getX())));
            telemetry.addLine(String.valueOf((int) follower.getPose().getY()));
            telemetry.addLine(String.valueOf((int) Math.toDegrees(follower.getHeading())));
            t.update();
        }
        tt.turretRegulator.interrupt();
        ll.startOrStopLL(true);

        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathSecondPresentArtefacts;
        public final PathChain PathSecondIntakingArtefacts;
        public final PathChain PathThirdScoring;
        public final PathChain OpenPath;
        public final Pose scoringPath = new Pose(101, 111);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0)) //was tangent

                    .build();
            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 84) // 93,95
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 84),

                                    new Pose(125, 84) // 106, 95
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            OpenPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 84),

                                    new Pose(133, 72)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                    .build();


            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133, 72),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))

                    .build();

            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 56) //93,72
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 56),

                                    new Pose(126, 56) //106, 72
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126, 56),

                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();


            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 123)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

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
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
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
                    follower.followPath(paths.OpenPath, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT) break;
                sh.openTunnel();
                setPathState(8);

                break;

            case 8:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1500) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.PathSecondPresentArtefacts, true);
                setPathState(9);
                break;

            case 9:

                if (!follower.isBusy()) {
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
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(12);
                break;

            case 12:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1500) break;
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
        timer.reset();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}
