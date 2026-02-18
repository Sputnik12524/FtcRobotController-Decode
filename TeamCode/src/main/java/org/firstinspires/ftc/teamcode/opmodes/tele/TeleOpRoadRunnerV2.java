package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.IOException;

@TeleOp(name = "TeleOpRRV2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    GamepadManager g1;
    GamepadManager g2;
    Shooter sh;
    Intake in;
    Turret tt;
    Logger lg;
    ElapsedTime timer;
    Transfer tr;
    Follower follower;

    boolean canStart = true;

    /// Intake
    boolean isRotateIn = false;
    boolean isShootingShort = false;
    boolean isShootingLong = false;
    boolean isRotateOut = false;

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean attentionControl = true;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        timer = new ElapsedTime();
        g1 = new GamepadManager(gamepad1);
        g2 = new GamepadManager(gamepad2);
        tr = new Transfer(this);
        sh = new Shooter(this, follower, tr);
        in = new Intake(this);
        tt = new Turret(this);

        try {
            lg.getAll("pospos");
        } catch (IOException | NullPointerException e) {
            e.printStackTrace();
            canStart = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
        }
        if (canStart) follower.setStartingPose(new Pose(lg.x, lg.y, lg.degrees));

        //currentPose = follower.getPose();
        isShootingLong = false;
        isShootingShort = false;
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);
        sh.closeTunnel();

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

            if (g1.A.isPressed() && !isRotateIn) {
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (g1.A.isPressed() && isRotateIn) {
                in.rotateStop();
                isRotateIn = false;
            }
            if (g1.B.isPressed() && !isRotateOut) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
            } else if (g1.B.isPressed() && isRotateOut) {
                in.rotateStop();
                isRotateOut = false;
            }

            //------------------------------------ SHOOTER
            if (!attentionControl) {
                sh.threeArtefactsShooting();
            }
            if (!attentionControl) {
                if (g2.X.isPressed()) sh.canShoot = true;
            }


            /// -------------------------------------- ПЕРЕКЛЮЧЕНИЕ ЭКСТРЕННОГО УПРАВЛЕНИЕ

            if (g2.dpadUp.isHeldFor(2500)) {
                attentionControl = true;
            }
            if (g2.dpadDown.isHeldFor(2500)) {
                attentionControl = false;
            }
            /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

            //---------------------------------------- TURRET

            if (attentionControl) {
                if (g1.Y.isPressed() && !isShootingLong) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                    sh.setLongThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = true;
                    isShootingShort = false;
                } else if (g1.X.isPressed() && !isShootingShort) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                    sh.setShortThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = false;
                    isShootingShort = true;
                } else if ((g1.Y.isPressed() && isShootingLong) || (g1.X.isPressed() && isShootingShort)) {
                    sh.closeTunnel();
                    sh.shootStop();
                    isShootingLong = false;
                    isShootingShort = false;
                }
            }

            if (attentionControl) {
                if (g1.dpadUp.isPressed()) {
                    sh.openTunnel();
                } else if (g1.dpadDown.isPressed()) {
                    sh.closeTunnel();
                }
            }

            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("Velocity shooter", sh.getVelocityRPS());
            telemetry.addData("TARGET", sh.velocityTarget);
            telemetry.addData("Empty", tr.isEmpty());
            telemetry.addData("0", tr.getColor().get(0));
            telemetry.addData("1", tr.getColor().get(1));
            telemetry.addData("2", tr.getColor().get(2));
            telemetry.update();
        }
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}