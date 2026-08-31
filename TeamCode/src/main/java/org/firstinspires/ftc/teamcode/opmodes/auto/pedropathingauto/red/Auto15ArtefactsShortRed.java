package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = ".RED 15 Short WITH AUTOAIMING", group = "Autonomous")
public class Auto15ArtefactsShortRed extends LinearOpMode {
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
    public static final double TURRET_WAIT = 3500;


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
        sh.setVelocityTarget(50);
        as.setAlliance(Alliance.RED);
        tt.turretRegulator.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        double lastVelo = 0;

        waitForStart();
        while (opModeIsActive()) {
            ll.update();
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose

            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double head = follower.getPose().getHeading();

            as.continuousCalculateGeneralValues(x, y, head, lastVelo);

            as.continuousSetAngleByInterpol();

            as.continuousTurnTurretToGate(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading()
            );
            sh.shootByVelocity();
            lastVelo = sh.getAngleAdjusterPos();


//            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("Aim  method", tt.getAimMethod());
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.addData("Turret heading", tt.getCurrentPosOfTurret());
            t.addData("turret target", as.target);
            t.update();

            if (loggerTimer.milliseconds() > 750) {
                lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
                loggerTimer.reset();
            }
        }

        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
        lg.fileClose();
        tt.turretRegulator.interrupt();
    }


    public static class Paths {
        public final PathChain PathFirstScoring;
        public final PathChain FifthPathScoring;
        public final PathChain SecondPathToPresetArtifacts, SecondPathIntakingArtifacts, SecondPathScoring;
        // public final PathChain OpenPath2;
        public final PathChain ThirdPathToGate, FourthPathIntaking, ThirdPathScoring;
        public final PathChain FourthPathScoring, FifthPathToPreset, FifthPathIntaking, PathToIntake;

        //        public final Pose forGateNym = new Pose(123, 67);
        public final Pose gateNym = new Pose(130, 60);
        final Pose scoringPath = new Pose(92, 92);


        public Paths(Follower follower) {
            PathFirstScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0)) //was tangent

                    .build();
            SecondPathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 58)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            SecondPathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 58),

                                    new Pose(127, 52) //106, 72
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();


            SecondPathScoring = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127, 52),

                                    new Pose(95, 57),


                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    scoringPath,

                                    new Pose(83, 31),

                                    new Pose(135, 73),

                                    gateNym
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathScoring = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateNym,

                                    new Pose(95, 57),


                                    scoringPath //106, 72
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            FourthPathIntaking = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    scoringPath,

                                    new Pose(95, 57),

                                    gateNym
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            FourthPathScoring = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateNym,

                                    new Pose(95, 57),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            PathToIntake = follower.pathBuilder().addPath(
                    new BezierLine(
                            gateNym,
                            new Pose(135, 56)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45)).build();

            FifthPathToPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            FifthPathIntaking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 84),

                                    new Pose(123, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            FifthPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(85, 104)
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
                follower.followPath(paths.PathFirstScoring);
                setPathState(1);
                break;

            case 1:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
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
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(8);

                break;

            case 8:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.ThirdPathToGate, true);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathToIntake);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    in.rotateIn();
                    follower.followPath(paths.ThirdPathScoring, true);
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
                if (follower.isBusy()) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.FourthPathIntaking);
                setPathState(28);
                break;


            case 28:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    follower.followPath(paths.PathToIntake);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    follower.followPath(paths.FourthPathScoring);
                    setPathState(14);
                }
                break;

            case 14:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(15);

                break;

            case 15:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 1000) {
                    follower.followPath(paths.FifthPathToPreset);
                    setPathState(16);
                }
                break;

            case 16:
                if (follower.isBusy()) break;
                sh.closeTunnel();
                follower.followPath(paths.FifthPathIntaking);
                setPathState(17);
                break;

            case 17:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                in.rotateIn();
                follower.followPath(paths.FifthPathScoring);
                setPathState(19);
                break;

            case 19:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(20);

                break;

            case 20:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1000) break;
                as.enableAutoTurretAiming(false);
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
