package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "RED 3 Short", group = "Autonomous")
@Configurable // Panels
public class Auto3ArtifactsShortRed extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();
        loggerTimer = new ElapsedTime();

        in = new Intake(this);
        sh = new Shooter(this);
        lg = new Logger("pospos");

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121, 127, Math.toRadians(-136)));

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setLongThrowMode();
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);

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
            t.addData("Shooter Velo", sh.getVelocityRPS());
            t.update();

            if (loggerTimer.milliseconds() > 300) {
                lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), 0);
                loggerTimer.reset();
            }
        }
        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), 0);
        lg.fileClose();
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121, 127),

                                    new Pose(101, 111)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136)) //was tangent

                    .build();
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103, 111),
                                    new Pose(85, 125)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-136)) //was tangent

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.PathScoring);
                sh.closeTunnel();
                sh.shootByVelocity();
                in.rotateIn();
                setPathState(1);
                break;
            case 1:
                if (!sh.isSpinUp() || follower.isBusy()) break;
                //while (!sh.isSpinUp() || follower.isBusy());
                sh.openTunnel();
                setPathState(2);
                break;
            case 2:
                if(follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                // while (follower.isBusy() || actionTimer.getElapsedTime() < 4000) ;
                sh.closeTunnel();
                follower.followPath(paths.PathLeaving);
                sh.shootStop();
                in.rotateStop();
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy() && !sh.isTunnelOpen) {
                    setPathState(-100);
                }
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}