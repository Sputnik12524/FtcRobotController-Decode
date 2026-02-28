package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


@TeleOp(name = "TeleOpRR", group = "0")
@Config
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter sh;
    Intake in;
    Turret tt;
    Transfer tr;
    Follower follower;
    AutoSniper as;
    Logger logger;
    Limelight ll;


    boolean wroteLogger = true;
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
    boolean attentionControl = true;
    public static double BLUE_ANGLE = 110;
    public static double RED_ANGLE = -120;
    public double shortBonusVelocity = 0;
    public double longBonusVelocity = 0;


    @Override
    public void runOpMode() throws InterruptedException {
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

        ll.startOrStopLL(false);

        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.degrees));
            if (logger.al == Alliance.BLUE) {
                as.setAlliance(Alliance.BLUE);
            } else as.setAlliance(Alliance.RED);
        } catch (IOException | NullPointerException e) {
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
            as.setAlliance(Alliance.RED);
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

            //------------------------------------- DRIVETRAIN
            follower.update();

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
            if (!attentionControl) sh.threeArtefactsShooting();
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
            if (!attentionControl)
                as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());


            /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

            if (g1.dpadLeft.isHeldFor(1500) && !attentionControl) {
                attentionControl = true;
                tt.turnByTarget(0);
            } else if (g1.dpadRight.isHeldFor(1500) && attentionControl) {
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
            if (g2.dpadUp.isHeldFor(1500)) {
                tt.turnByTarget(0);
                if (!wroteLogger) follower.setPose(new Pose(72, 72, 0));
                if (as.alliance == Alliance.RED) follower.setPose(new Pose(135, 7, 180));
                if (as.alliance == Alliance.BLUE) follower.setPose(new Pose(11, 7, 0));
            }


            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("Alliance", as.alliance); //не нужно
            telemetry.addData("TARGET", sh.velocityTarget / 28); // не нужно
            telemetry.addData("Velocity", sh.getVelocityRPS());
            telemetry.addData("InZone", sh.inZone());
            telemetry.addData("howMany", tr.howMany());
            telemetry.addLine(String.valueOf((int) (follower.getPose().getX())));
            telemetry.addLine(String.valueOf((int) follower.getPose().getY()));
            telemetry.addLine(String.valueOf((int) follower.getHeading())); //  нужно

            dashtele.addData("Target ", sh.velocityTarget / 28);
            dashtele.addData("Velocity shooter", sh.getVelocityRPS());
            dashtele.addData("ADJUSTER POS", sh.angleAdjuster.getPosition());
            dashtele.update();
            telemetry.update();
        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
    // 135 7 180  11 7 0

}