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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "3 Artifacts Short Auto RED", group = "Autonomous")
@Configurable // Panels
public class Auto3ArtifactsShortRed extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot
    Turret tt;

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
        tt = new Turret(this);
        Logger lg = new Logger("pospos");

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 123, Math.toRadians(35)));//22,124

        paths = new Paths(follower); // Build paths

        sh.closeTunnel();
        sh.setLongThrowMode();
        tt.turretRegulator.start();
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
            t.addData("Shooter Velo", sh.getVelocityRPS());
            t.update();
        }
        lg.writePose(Alliance.RED, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
        tt.turretRegulator.interrupt();
    }


    public static class Paths {
        public PathChain PathScoring, PathLeaving;

        public Paths(Follower follower) {
            PathScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123, 123),

                                    new Pose(100, 102)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 102),
                                    new Pose(100, 18)
                            )
                    ).setTangentHeadingInterpolation()

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
                tt.continuousTurnToGate(Alliance.RED, follower.getPose().getX(),
                        follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
                setPathState(1);
                break;
            case 1:
                while (!sh.isSpinUp() || follower.isBusy());
                sh.openTunnel();
                setPathState(2);
                break;
            case 2:
                while (follower.isBusy() || actionTimer.getElapsedTime() < 4000) ;
                sh.closeTunnel();
                follower.followPath(paths.PathScoring);
                sh.shootStop();
                in.rotateStop();
                tt.turnByTarget(0);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy() && !Shooter.isTunnelOpen && tt.getCurrentPosOfTurret() == 0) {
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
