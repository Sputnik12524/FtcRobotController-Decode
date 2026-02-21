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


@TeleOp(name = "TeleOpRR", group = "0")
@Config
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter sh;
    Intake in;
    Turret tt;
    ElapsedTime timer;
    Transfer tr;
    Follower follower;

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


    @Override
    public void runOpMode() throws InterruptedException {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);
        follower = Constants.createFollower(hardwareMap);
        timer = new ElapsedTime();
        tr = new Transfer(this);
        sh = new Shooter(this, follower, tr);
        in = new Intake(this);
        tt = new Turret(this);

        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

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


            if (gamepad1.y && !isShootingLong && !stateY1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShootingLong = true;
                isShootingShort = false;
            } else if (gamepad1.x && !isShootingShort && !stateX1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
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

            if (g1.dpadLeft.isHeldFor(1500) && !attentionControl) {
                attentionControl = true;
            } else if (g1.dpadLeft.isHeldFor(1500) && attentionControl) {
                attentionControl = false;
            }

            if (gamepad2.aWasPressed()) {
                tt.turnByTarget(180);            }

            if (attentionControl) {
                if (gamepad1.dpad_up) {
                    sh.openTunnel();
                } else if (gamepad1.dpad_down) {
                    sh.closeTunnel();
                }
            }


            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("TARGET", sh.velocityTarget);
            telemetry.addData("Human", sh.ifNotInLaunchZoneHuman());
            telemetry.addData("Goal", sh.ifNotInLaunchZoneGoal());
            telemetry.addData("InZone", sh.inZone());
            telemetry.addData("howMany", tr.howMany());

            dashtele.addData("Target ", sh.velocityTarget);
            dashtele.addData("Velocity shooter", sh.getVelocityRPS());
            dashtele.addData("ADJUSTER POS", sh.angleAdjuster.getPosition());
            dashtele.update();
            telemetry.update();
        }

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

}