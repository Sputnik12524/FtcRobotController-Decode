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
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous(name = "BLUE 3 Long", group = "Autonomous")
public class Auto3ArtefactsLongBlue extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer;
    private Timer actionTimer;
    public Pose currentPose; // Current pose of the robot

    Intake in;
    Shooter sh;
    Logger lg;
    Turret tt;
    Limelight ll;
    AutoSniper as;
    private Paths paths; // Paths defined in the Paths class



    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        in = new Intake(this);
        sh = new Shooter(this);
        lg = new Logger("pospos");
        ll = new Limelight(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh, ll, follower);

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dash);

        // Paths defined in the Paths class
        paths = new Paths(follower); // Build paths

        sh.openTunnel();
        sh.setLongThrowMode();
        tt.turretRegulator.start();
        ll.startOrStopLL(false);
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);

        waitForStart();
        while (opModeIsActive()) {
            ll.update();
            follower.update();
            autonomousPathUpdate();
            currentPose = follower.getPose();

            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

            t.addData("Path State", pathState);
            t.addData("X", follower.getPose().getX());
            t.addData("Y", follower.getPose().getY());
            t.addData("Heading", follower.getPose().getHeading());
         //   t.addData("Shooter Velocity", sh.getVelocityRPS());
            t.update();
        }
        tt.turretRegulator.interrupt();
        ll.startOrStopLL(true);
        lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        lg.fileClose();
    }

    public static class Paths {
        public final PathChain PathLeaving;

        public Paths(Follower follower) {
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56, 8),

                                    new Pose(56, 27)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(actionTimer.getElapsedTime() < 15000) break;

                setPathState(1);
            case 1:
                sh.closeTunnel();
                sh.shootByVelocity();
                in.rotateIn();
                setPathState(2);
                break;
            case 2:
                if (!sh.isSpinUp()||follower.isBusy())  break;
                sh.openTunnel();
                setPathState(3);
                break;
            case 3:
                if (follower.isBusy() || actionTimer.getElapsedTime() < 4000) break;
                tt.turnByTarget(0);
                follower.followPath(paths.PathLeaving);
                setPathState(4);
                break;
            case 4:
                if(follower.isBusy()) break;
                sh.closeTunnel();
                sh.shootStop();
                in.rotateStop();
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy() && !sh.isTunnelOpen) {
                    lg.writePose(Alliance.BLUE, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
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


