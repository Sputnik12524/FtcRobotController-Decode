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
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "Long 9 BLUE ", group = "Autonomous")
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

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();
        Transfer tr = new Transfer(this);

        in = new Intake(this);
        sh = new Shooter(this, follower, tr);
        lg = new Logger("pospos");

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(-90)));

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setLongThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose



            // Log values to Panels and Driver Station
            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
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
        public final Pose scoringPose = new Pose(60, 12);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60, 8),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(116))

                    .build();
            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(47, 30) //ровно в педре на 35 y
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(-180))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 30),

                                    new Pose(20, 30)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 30),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(116))

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


            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(5, 12)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(-180))

                    .build();

            //    Отъехать назад и снова вперед, чтобы точно захватить

            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(5, 12),

                                    new Pose(5, 4)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(-180))

                    .build();

            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(5, 4),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(110))

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

                                    new Pose(35, 28)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(110))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.openTunnel();
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.shootByVelocity();
                follower.followPath(paths.PathScoring);
                setPathState(1);
                break;

            case 1:
                if (!sh.isSpinUp()||follower.isBusy()) break;
                in.rotateIn();
                setPathState(2);
                break;

            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                in.rotateStop();
                sh.closeTunnel();
                follower.followPath(paths.PathToPresetArtifacts);
                in.rotateIn();
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathIntakingArtifacts, true);
                    in.rotateStop();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondScoring, true);
                    sh.openTunnel();
                    setPathState(6);
                }
                break;

            case 6:
                if (!sh.isSpinUp()||follower.isBusy()) break;
                in.rotateIn();
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondPresentArtefacts, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (follower.isBusy()) break;
                sh.closeTunnel();
                in.rotateStop();
                follower.followPath(paths.PathSecondIntakingArtefacts, true);
                in.rotateIn();
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathThirdScoring, true);
                    setPathState(10);
                }
                break;


            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondIntakingArtefacts, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!sh.isSpinUp()||follower.isBusy()) break;
                in.rotateIn();
                setPathState(12);
                break;

            case 12:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                in.rotateStop();
                sh.closeTunnel();
                follower.followPath(paths.PathLeaving);
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
