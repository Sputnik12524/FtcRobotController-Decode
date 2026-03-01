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
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "9 Short RED", group = "Autonomous")
public class Auto9ArtefactsShortRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Logger lg;
    Transfer tr;
    Turret tt;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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
        sh = new Shooter(this, follower, tr);
        lg = new Logger("pospos");
        AutoSniper as = new AutoSniper(tt,sh);
        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        as.setAlliance(Alliance.RED);
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
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
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;
        public final PathChain SecondPathToPresetArtifacts;
        public final PathChain SecondPathIntakingArtifacts;
        public final PathChain SecondPathScoring;
        public final PathChain ThirdPathPresetArtefacts;
        public final PathChain ThirdPathIntakingArtefacts;
        public final PathChain ThirdPathScoring;
        public final Pose scoringPath = new Pose(101, 111);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 122),

                                    scoringPath
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136)) //was tangent

                    .build();
            SecondPathToPresetArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 84) // 93,95
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();
            SecondPathIntakingArtifacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 84),

                                    new Pose(125, 84) // 106, 95
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            SecondPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 84),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();

            ThirdPathPresetArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 60) //93,72
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-136), Math.toRadians(0))

                    .build();

            ThirdPathIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 60),

                                    new Pose(120, 60) //106, 72
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ThirdPathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120, 60),

                                    scoringPath
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-136))

                    .build();


            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPath,

                                    new Pose(100, 123)
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
                if (!sh.isSpinUp()||follower.isBusy()) break;
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
                if (!sh.isSpinUp()||follower.isBusy()) break;
                sh.openTunnel();
                setPathState(7);

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
                if (!sh.isSpinUp()||follower.isBusy()) break;
                sh.openTunnel();
                setPathState(11);
                break;

            case 11:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                tt.turnByTarget(0);
                in.rotateStop();
                sh.shootStop();
                follower.followPath(paths.PathLeaving);
                lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
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
