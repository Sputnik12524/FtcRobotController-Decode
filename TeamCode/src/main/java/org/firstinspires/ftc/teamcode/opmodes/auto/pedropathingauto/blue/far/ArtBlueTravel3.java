package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto.blue.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "С проездом BLUE 3", group = "Autonomous")
@Disabled
@Configurable // Panels
public class ArtBlueTravel3 extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Transfer tr;
    Logger lg;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        in = new Intake(this);
        sh = new Shooter(this);
        lg = new Logger("pospos");

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));//22,124

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setShortThrowMode();
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
            t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
    }


    public static class Paths {
        public final PathChain PathScoring;
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60, 8),

                                    new Pose(60, 104)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-36))

                    .build();
            PathLeaving = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60,104),
                            new Pose(26,88)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(-36)).build();
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
                follower.followPath(paths.PathLeaving);
                sh.closeTunnel();

                sh.shootStop();
                in.rotateStop();
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy() && !sh.isTunnelOpen ) {
                    lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    lg.fileClose();
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
