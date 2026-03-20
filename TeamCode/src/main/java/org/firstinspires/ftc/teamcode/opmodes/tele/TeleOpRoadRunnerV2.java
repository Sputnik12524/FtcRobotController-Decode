package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.IOException;


@TeleOp(name = "TeleOpRR_V2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    Paths paths;
    Shooter sh;
    Intake in;
    Turret tt;
    Transfer tr;
    Follower follower;
    AutoSniper as;
    Logger logger;
    Limelight ll;
    ElapsedTime timer;

    int pathState;

    enum MODE {AUTO, DRIVER}

    MODE mode = MODE.DRIVER;


    boolean wroteLogger = true;
    boolean isPoseReset = false;
    Alliance alliance;
    boolean complete = false;
    /// Intake
    boolean isRotateIn = false;
    boolean isShootingShort = false;
    boolean isShootingLong = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean attentionControl = false;
    public double shortBonusVelocity = 0;
    public double longBonusVelocity = 0;
    boolean isSpinUp = false;


    @Override
    public void runOpMode() {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);
        follower = Constants.createFollower(hardwareMap);
        tr = new Transfer(this);
        ll = new Limelight(this);
        sh = new Shooter(this, follower, tr);
        in = new Intake(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh);
        logger = new Logger("pospos");
        paths = new Paths(follower);
        timer = new ElapsedTime();


        ll.startOrStopLL(false);

        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.degrees));
            if (logger.al == Alliance.BLUE) {
                alliance = Alliance.BLUE;
                as.setAlliance(Alliance.BLUE);
            } else {
                as.setAlliance(Alliance.RED);
                alliance = Alliance.RED;
            }
        } catch (IOException | NullPointerException e) {
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
            as.setAlliance(Alliance.NONE);
        }

        follower.update();

        if (wroteLogger)
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

        isShootingLong = false;
        isShootingShort = false;
        DriveTrain dt = new DriveTrain(this);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        sh.closeTunnel();
        tt.turnByTarget(0);
        tt.turretRegulator.start();

        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            g1.update();
            g2.update();
            follower.update();
            if (!attentionControl) sh.threeArtefactsShooting();
            if (!attentionControl)
                as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

            if (mode == MODE.DRIVER) {

                if (g1.dpadUp.isHeldFor(1000)) {
                    mode = MODE.AUTO;
                    setPathState(0);
                }

                if (gamepad2.aWasPressed()) {
                    setPathState(0);
                }

                //------------------------------------- DRIVETRAIN


                if (gamepad1.right_bumper) {
                    dt.turnRightSlowMode();
                } else if (gamepad1.left_bumper) {
                    dt.turnLeftSlowMode();
                } else {
                    dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
                }


                //-------------------------------------- INTAKE

                if (gamepad1.a && !isRotateIn && !stateA1) {
                    in.rotateIn();
                    isRotateIn = true;
                    isRotateOut = false;
                } else if (gamepad1.a && isRotateIn && !stateA1) {
                    in.rotateStop();
                    isRotateIn = false;
                }
                if (gamepad1.b && !isRotateOut && !stateB1) {
                    in.rotateOut();
                    isRotateOut = true;
                    isRotateIn = false;
                } else if (gamepad1.b && isRotateOut && !stateB1) {
                    in.rotateStop();
                    isRotateOut = false;
                }
                stateA1 = gamepad1.a;
                stateB1 = gamepad1.b;


                //------------------------------------ SHOOTER
                if (!attentionControl) if (gamepad1.dpad_up) sh.canShoot = true;

                if (gamepad2.yWasPressed()) (shortBonusVelocity) += 0.75;
                if (gamepad2.xWasPressed()) (shortBonusVelocity) -= 0.75;
                if (gamepad2.bWasPressed()) (longBonusVelocity) += 0.75;
                if (gamepad2.aWasPressed()) (longBonusVelocity) -= 0.75;


                if (gamepad1.y && !isShootingLong && !stateY1) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW + longBonusVelocity);
                    sh.setLongThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = true;
                    isShootingShort = false;
                } else if (gamepad1.x && !isShootingShort && !stateX1) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW + shortBonusVelocity);
                    sh.setShortThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = false;
                    isShootingShort = true;
                } else if ((gamepad1.y && !stateY1 && isShootingLong) || (gamepad1.x && !stateX1 && isShootingShort)) {
                    sh.closeTunnel();
                    sh.shootStop();
                    isShootingLong = false;
                    isShootingShort = false;
                }
                stateY1 = gamepad1.y;
                stateX1 = gamepad1.x;

                //---------------------------------------- TURRET


                /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

                if (g1.dpadRight.isPressed() && !attentionControl && g1.dpadRight.getToggleState()) {
                    attentionControl = true;
                    tt.turnByTarget(0);
                } else if (g1.dpadRight.isPressed() && attentionControl && !g1.dpadRight.getToggleState()) {
                    attentionControl = false;
                }

                if (gamepad2.yWasPressed()) {
                    tt.turnByTarget(0);
                }

                if (attentionControl) {
                    if (gamepad1.dpad_up) {
                        sh.openTunnel();
                    } else if (gamepad1.dpad_down) {
                        sh.closeTunnel();
                    }
                }
                if (g2.dpadUp.isPressed()) {
                    tt.turnByTarget(0);
                    isPoseReset = true;
                    follower.setPose(new Pose(135, 7, Math.toRadians(180)));
                    as.setAlliance(Alliance.BLUE);
                }

                if (g2.dpadDown.isPressed()) {
                    isPoseReset = true;
                    tt.turnByTarget(0);
                    follower.setPose(new Pose(11, 7, 0));
                    as.setAlliance(Alliance.RED);
                }

                if (gamepad2.leftBumperWasPressed()) {
                    as.targetBonus += 5;
                }
                if (gamepad2.rightBumperWasPressed()) {
                    as.targetBonus -= 5;
                }
            } else {
                pathsUpdate();
                if (complete) mode = MODE.DRIVER;
                if (g1.dpadUp.isHeldFor(1000)) mode = MODE.DRIVER;
            }


            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("Velocity", sh.getVelocityRPS());
            telemetry.addData("InZone", sh.inZone());
            telemetry.addData("howMany", tr.howMany());
            // telemetry.addData("Позиция сброшена", isPoseReset);
            telemetry.addData("Alliance", as.alliance);
            telemetry.addData("TARGET", sh.velocityTarget / 28);
            telemetry.addLine(String.valueOf((int) (follower.getPose().getX())));
            telemetry.addLine(String.valueOf((int) follower.getPose().getY()));
            telemetry.addLine(String.valueOf((int) Math.toDegrees(follower.getHeading())));


            dashtele.addData("Target ", sh.velocityTarget / 28);
            dashtele.addData("Velocity shooter", sh.getVelocityRPS());
            dashtele.addData("ADJUSTER POS", sh.angleAdjuster.getPosition());
            dashtele.update();
            telemetry.update();
        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }

    public static class Paths {
        public final PathChain blueParking;
        public final PathChain redParking;
        public final PathChain blueHuman;
        public final PathChain redHuman;
        public final PathChain blueGoal;
        public final PathChain redGoal;

        public Paths(Follower follower) {
            blueParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(105, 33))
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            redParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(38.5, 33))
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            blueHuman = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(92, 8)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            redHuman = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(54, 6)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            blueGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(42, 105)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            redGoal = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
                            new Pose(102, 105)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }

    }

    public void pathsUpdate() {
        switch (pathState) {
            case 0:
                if (alliance == Alliance.BLUE) follower.followPath(paths.blueParking);
                else if (alliance == Alliance.RED) follower.followPath(paths.redParking);
                if (!follower.isBusy()) complete = true;
                break;
            case 1:
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.closeTunnel();
                in.rotateStop();
                if (alliance == Alliance.BLUE) follower.followPath(paths.blueHuman);
                else if (alliance == Alliance.RED) follower.followPath(paths.redHuman);
                if (sh.isSpinUp()) {
                    timer.reset();
                    in.rotateIn();
                    sh.openTunnel();
                    isSpinUp = true;
                }
                if (!follower.isBusy() && timer.milliseconds() > 500 && isSpinUp){
                    complete = true;
                    isSpinUp = false;
                }
                break;

            case 3:
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.closeTunnel();
                in.rotateStop();
                if (alliance == Alliance.BLUE) follower.followPath(paths.blueGoal);
                else if (alliance == Alliance.RED) follower.followPath(paths.redGoal);
                if (sh.isSpinUp()) {
                    timer.reset();
                    in.rotateIn();
                    sh.openTunnel();
                    isSpinUp = true;
                }
                if (!follower.isBusy() && timer.milliseconds() > 500 && isSpinUp){
                    complete = true;
                    isSpinUp = false;
                }
                break;

        }

    }

    public void setPathState(int state) {
        pathState = state; //добавить таймер
        mode = MODE.AUTO;
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}