package org.firstinspires.ftc.teamcode.opmodes.auto.pedropathingauto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "3 Artifacts Long Autonomous BLUE", group = "Autonomous")
@Configurable // Panels
public class Auto3ArtifactsLongBlue extends LinearOpMode {
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

        // Panels Telemetry instance
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 124, Math.toRadians(110)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        sh.closeTunnel();
        sh.setShortThrowMode();
        tt.turretRegulator.start();

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine
            currentPose = follower.getPose(); // Update the current pose


            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
        tt.turretRegulator.interrupt();
    }


    public static class Paths {
        public PathChain PathLeaving;

        public Paths(Follower follower) {
            PathLeaving = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60, 18),

                                    new Pose(20, 15)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))

                    .build();
        }

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.closeTunnel();
                sh.shootByVelocity();
                in.rotateIn();
                tt.continuousTurnToGate(Alliance.BLUE, follower.getPose().getX(),
                        follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
                setPathState(1);
                break;
            case 1:
                if (sh.getVelocityRPS() >= Shooter.VELOCITY_FOR_LONG_THROW && !follower.isBusy()) {
                    sh.openTunnel();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && Shooter.isTunnelOpen && actionTimer.getElapsedTime() >= 10000) {
                    sh.closeTunnel();
                    follower.followPath(paths.PathLeaving);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && !Shooter.isTunnelOpen){
                    setPathState(-100);
                }
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
