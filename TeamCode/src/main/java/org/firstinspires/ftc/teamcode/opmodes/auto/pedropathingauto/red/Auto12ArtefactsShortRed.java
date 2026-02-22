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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "12 Short RED ", group = "Autonomous")
public class Auto12ArtefactsShortRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        in = new Intake(this);
        sh = new Shooter(this);
        Logger lg = new Logger("pospos");

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 122, Math.toRadians(-136)));

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose


            sh.threeArtefactsShooting();

            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain SecondPathToPresetArtifacts, SecondPathIntakingArtifacts, SecondPathScoring;
        public final PathChain ThirdPathPresetArtefacts, ThirdPathIntakingArtefacts, ThirdPathScoring;
        public final PathChain FourthPathToPreset, FourthPathIntaking, FourthPathScoring;
        Pose scoringPose = new Pose(101, 111);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136))

                    .build();
            SecondPathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,
                                    new Pose(93, 95)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();
            SecondPathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93, 95),

                                    new Pose(106, 95)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            SecondPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(106, 95),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();

            ThirdPathPresetArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(93, 72)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();

            ThirdPathIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93, 72),

                                    new Pose(106, 72)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(106, 72),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();

            FourthPathToPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(93, 48)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();

            FourthPathIntaking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93, 48),

                                    new Pose(106, 48)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            FourthPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(106, 48),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();

            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(125, 103)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136))

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
                if (!sh.isSpinUp()) break;
                sh.openTunnel();
                setPathState(2);
                break;
            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                follower.followPath(paths.SecondPathToPresetArtifacts);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    in.rotateIn();
                    sh.closeTunnel();
                    follower.followPath(paths.SecondPathIntakingArtifacts, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SecondPathScoring, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (sh.isSpinUp()) {
                    sh.openTunnel();
                    setPathState(7);
                }
                break;

            case 7:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.ThirdPathPresetArtefacts, true);
                setPathState(8);

                break;

            case 8:

                if (!follower.isBusy()) {
                    follower.followPath(paths.ThirdPathIntakingArtefacts, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ThirdPathScoring, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!sh.isSpinUp()) break;
                sh.openTunnel();
                setPathState(11);
                break;

            case 11:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.FourthPathToPreset);
                setPathState(12);

                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.FourthPathIntaking);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.FourthPathScoring);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() && sh.isSpinUp()) {
                    sh.openTunnel();
                    setPathState(15);
                }
                break;
            case 15:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                follower.followPath(paths.PathLeaving);
                setPathState(-100);
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}
