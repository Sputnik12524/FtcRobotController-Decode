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
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "BLUE 9 Short  ", group = "Autonomous")
public class Auto9ArtefactsShortBlue extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    ElapsedTime timer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Logger lg;
    AutoSniper as;
    Turret tt;
    Limelight ll;

    public static double TURRET_WAIT = 3500;

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
        follower.setStartingPose(new Pose(22, 127, Math.toRadians(-36))); //Math.toRadians(-36)));


        in = new Intake(this);
        ll = new Limelight(this);
        lg = new Logger("pospos");
        tt = new Turret(this, ll);
        sh = new Shooter(this, follower);
        as = new AutoSniper(tt, sh, ll);
        timer = new ElapsedTime();

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setShortThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW - 6.5);
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();
        ll.startOrStopLL(false);

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
            t.addData("Aim method", tt.getAimMethod());
            t.update();
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
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
        public final Pose scoringPose = new Pose(43, 120);


        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22, 127),

                                    scoringPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))

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
                                    new Pose(20, 84), //35,100

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))//-36

                    .build();

            PathSecondPresentArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(50, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathSecondIntakingArtefacts = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50, 60),

                                    new Pose(23, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();

            PathThirdScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35, 60),

                                    scoringPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-180))

                    .build();


            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    scoringPose,

                                    new Pose(42, 130) //
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
                    follower.followPath(paths.PathSecondScoring, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(7);

                break;

            case 7:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 1500) break;
                sh.closeTunnel();
                in.rotateIn();
                follower.followPath(paths.PathSecondPresentArtefacts, true);
                setPathState(8);
                break;

            case 8:

                if (!follower.isBusy()) {
                    follower.followPath(paths.PathSecondIntakingArtefacts, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PathThirdScoring, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!sh.isSpinUp() || follower.isBusy() || actionTimer.getElapsedTime() < TURRET_WAIT)
                    break;
                sh.openTunnel();
                setPathState(11);
                break;

            case 11:
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
