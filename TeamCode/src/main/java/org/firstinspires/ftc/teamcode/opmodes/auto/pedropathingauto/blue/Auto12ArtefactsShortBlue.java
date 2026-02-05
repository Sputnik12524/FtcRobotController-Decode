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
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "Auto short 12 BLUE ", group = "Autonomous")
public class Auto12ArtefactsShortBlue extends LinearOpMode {
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
        follower.setStartingPose(new Pose(21, 125, Math.toRadians(-40)));

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
        public final PathChain PathToPresetArtifacts;
        public final PathChain PathIntakingArtifacts;
        public final PathChain PathSecondScoring;
        public final PathChain PathSecondPresentArtefacts; //48.60
        public final PathChain PathSecondIntakingArtefacts;  //17.60
        public final PathChain PathThirdScoring; // 47.115
        public final PathChain PathThirdPresentArtefacts;
        public final PathChain PathThirdIntakingArtefacts;
        public final PathChain PathFourScorying;


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21, 125),

                                    new Pose(47, 111)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            PathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 111),

                                    new Pose(48, 95)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-180))

                    .build();
            PathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48, 95),

                                    new Pose(20, 95)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 95),

                                    new Pose(47, 111)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-40))

                    .build();

            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 111),

                                    new Pose(47, 65)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-180))

                    .build();

            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 65),

                                    new Pose(20, 65)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 65),

                                    new Pose(47, 111)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-40))

                    .build();


            PathThirdPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 111),

                                    new Pose(47, 43)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-180))

                    .build();

            PathThirdIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47, 43),

                                    new Pose(20, 43)
                            )

                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            PathFourScorying = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 43),

                                    new Pose(101, 111)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-40))

                    .build();


            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101, 111),

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
                //  sh.waitForShoot();
                setPathState(2);
                break;
            case 2:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
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
                if (!sh.isSpinUp()) {
                    sh.openTunnel();
                    // sh.waitForShoot();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondPresentArtefacts, true);
                    setPathState(8);
                }
                break;

            case 8:
                sh.closeTunnel();
                in.rotateIn();
                if (!follower.isBusy())
                    follower.followPath(paths.PathSecondIntakingArtefacts, true);
                setPathState(9);

            case 9:
                if (!follower.isBusy())
                    follower.followPath(paths.PathThirdScoring, true);
                setPathState(10);

            case 10:
                if (!sh.isSpinUp())
                    sh.openTunnel();
                setPathState(11);

            case 11:
                sh.closeTunnel();
                if(!follower.isBusy())
                    follower.followPath(paths.PathThirdPresentArtefacts, true);
                setPathState(12);

            case 12:
                in.rotateIn();
                if(!follower.isBusy())
                    follower.followPath(paths.PathThirdIntakingArtefacts, true);
                setPathState(13);

            case 13:
                if(!follower.isBusy())
                    follower.followPath(paths.PathFourScorying, true);
                setPathState(14);

            case 14:
                if(!follower.isBusy())
                    follower.followPath(paths.PathThirdIntakingArtefacts, true);
                setPathState(15);

            case 15:
                if (!sh.isSpinUp())
                    sh.openTunnel();
                setPathState(16);

            case 16:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                in.rotateStop();
                follower.followPath(paths.PathLeaving);
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
