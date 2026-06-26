package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue;

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

@Autonomous(name = "BLUE 12 short", group = "Autonomous")
public class Auto12ArtefactsShortBlue extends LinearOpMode {
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
    ElapsedTime loggerTimer;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();
        loggerTimer = new ElapsedTime();

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

            as.continuousTurnTurretToGate(x, y, head);
            as.continuousCalculateGeneralValues(x, y, head, lastVelo);
            as.continuousSetAngleByInterpol();

            lastVelo = sh.getVelocityRPS();
            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Turret ", tt.getCurrentPosOfTurret());
            // t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
            if (loggerTimer.milliseconds() > 750) {
                lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
                loggerTimer.reset();
            }
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), tt.getCurrentPosOfTurret());
        lg.fileClose();
        tt.turnByTarget(0);
        tt.turretRegulator.interrupt();
        ll.startOrStopLL(true);
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain SecondPathToPresetArtifacts;
        public final PathChain SecondPathIntakingArtifacts;
        public final PathChain SecondPathScoring;
        public final PathChain ThirdPathPresetArtefacts; //48.60
        public final PathChain ThirdPathIntakingArtefacts;  //17.60
        public final PathChain ThirdPathScoring; // 47.115
        public final PathChain FourthPathToPreset;
        public final PathChain FourthPathIntaking;
        public final PathChain FourthPathScoring;
        public final PathChain PathOpenGate;
        public final Pose scoringPose = new Pose(43, 120);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22, 127),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-180))

                    .build();

            SecondPathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(

                                    scoringPose,

                                    new Pose(50, 62) //55,100
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();
            SecondPathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50, 62), //55,100

                                    new Pose(15, 62) //35,100
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();


            PathOpenGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15, 62),

                                    new Pose(15, 62)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            SecondPathScoring = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15, 62), //35,100

                                    new Pose(59, 65),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180)) //-36

                    .build();

            ThirdPathPresetArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(38, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            ThirdPathIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38, 84),

                                    new Pose(14, 84)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            ThirdPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14, 84),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            FourthPathToPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(52, 38)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            FourthPathIntaking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52, 38),

                                    new Pose(15, 34)
                            )

                    ).setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();


            FourthPathScoring = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(25, 36),

                                    new Pose(47, 58),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

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
                follower.followPath(paths.SecondPathToPresetArtifacts);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    sh.closeTunnel();
                    follower.followPath(paths.SecondPathIntakingArtifacts, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    in.rotateStop();
                    follower.followPath(paths.PathOpenGate, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() || actionTimer.getElapsedTime() > 1000) {
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
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1400) break;
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
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1400) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.FourthPathToPreset);
                setPathState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.FourthPathIntaking);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.FourthPathScoring);
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
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1400) break;
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
