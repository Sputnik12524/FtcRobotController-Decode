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
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "RED 18 Short", group = "Autonomous")
public class Auto18ArtefactsShortRed extends LinearOpMode {
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
    Turret tt;
    AutoSniper as;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();
        loggerTimer = new ElapsedTime();

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 122, Math.toRadians(-136)));


        in = new Intake(this);
        Limelight ll = new Limelight(this);
        tt = new Turret(this, ll);
        sh = new Shooter(this);
        lg = new Logger("pospos");
        as = new AutoSniper(tt, sh, ll);


        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
        as.setAlliance(Alliance.RED);
        tt.turretRegulator.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        waitForStart();
        while (opModeIsActive()) {
            ll.update();
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose

            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

//            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.addData("Turret heading", tt.getCurrentPosOfTurret());
            t.addData("turret target", as.target);
            t.update();

            if (loggerTimer.milliseconds() > 300) {
                lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), 0);
                loggerTimer.reset();
            }
        }
        tt.turnByTarget(0);
        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
        lg.fileClose();
        tt.turretRegulator.interrupt();
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain SecondPathToPresetArtifacts, SecondPathIntakingArtifacts, SecondPathScoring;
        // public final PathChain OpenPath2;
        public final PathChain ThirdPathPresetArtefacts, ThirdPathIntakingArtefacts, ThirdPathScoring;
        public final PathChain GatePathToPreset, GatePathIntaking, GatePathScoring;
        public final Pose forGateNym = new Pose(123, 67);
        public final Pose gateNym = new Pose(132, 62);
        final Pose scoringPath = new Pose(101, 111);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0)) //was tangent

                    .build();
            SecondPathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            SecondPathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 84),

                                    new Pose(123, 84) // 106, 95
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();


            SecondPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133, 72),

                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathPresetArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 58)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 58),

                                    new Pose(127, 52) //106, 72
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127, 52),

                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            GatePathToPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    forGateNym
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33))

                    .build();

            GatePathIntaking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    forGateNym,

                                    gateNym
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(33))

                    .build();

            GatePathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateNym,

                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(33))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(125, 103)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(33))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                tt.turnByTarget(30);
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
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                follower.followPath(paths.SecondPathToPresetArtifacts);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    sh.closeTunnel();
                    follower.followPath(paths.SecondPathIntakingArtifacts, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SecondPathScoring, true);
                    in.rotateIn();
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
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.ThirdPathPresetArtefacts, true);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ThirdPathIntakingArtefacts, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    follower.followPath(paths.ThirdPathScoring, true);
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
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.GatePathToPreset);
                setPathState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GatePathIntaking);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.GatePathScoring);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    setPathState(16);
                }
                break;

            case 16:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.GatePathToPreset);
                setPathState(17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GatePathIntaking);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.GatePathScoring);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    setPathState(20);
                }
                break;

            case 20:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.GatePathToPreset);
                setPathState(21);
                break;

            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GatePathIntaking);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy() && actionTimer.getElapsedTime() < 1500) {
                    follower.followPath(paths.GatePathScoring);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    setPathState(24);
                }
                break;

            case 24:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                tt.turnByTarget(0);
                in.rotateStop();
                sh.shootStop();
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
