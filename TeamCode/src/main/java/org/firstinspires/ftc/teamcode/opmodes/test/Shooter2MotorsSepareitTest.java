package org.firstinspires.ftc.teamcode.opmodes.test;

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

import java.util.ArrayList;


@TeleOp(name = "Shooter motors SEPAREIT", group = "1")
//@Config5,k.l
public class Shooter2MotorsSepareitTest extends LinearOpMode {
    Shooter sh;
    Follower follower;

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean stateA1 = false;
    boolean attentionControl = true;
    boolean isShootingLong = false;


    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        sh = new Shooter(this);

        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        //currentPose = follower.getPose();
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

            //------------------------------------- DRIVETRAIN
            follower.update();

            if (gamepad1.right_bumper) {
                dt.turnRightSlowMode();
            } else if (gamepad1.left_bumper) {
                dt.turnLeftSlowMode();
            } else {
                dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }

            //------------------------------------ SHOOTER
            if (!attentionControl) {
                sh.threeArtefactsShooting();
                sh.switchCover();
            }
            if (!attentionControl) {
                if (gamepad2.x) sh.canShoot = true;
            }

            if (gamepad1.y && !isShootingLong && !stateY1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocityUpper();
                isShootingLong = true;
            } else if (gamepad1.y && !stateY1 && isShootingLong) {
                sh.closeTunnel();
                sh.shootStopUpper();
                isShootingLong = false;
            }

            if (gamepad1.x && !isShootingLong && !stateX1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocityLower();
                isShootingLong = true;
            } else if (gamepad1.x && !stateX1 && isShootingLong) {
                sh.closeTunnel();
                sh.shootStopLower();
                isShootingLong = false;
            }

            if (gamepad1.a && !isShootingLong && !stateA1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShootingLong = true;
            } else if (gamepad1.y && !stateY1 && isShootingLong){
                sh.closeTunnel();
                sh.shootStop();
                isShootingLong = false;
            }
            stateA1 = gamepad1.y;

            stateY1 = gamepad1.y;
            stateX1 = gamepad1.x;

            if (gamepad1.dpad_up) {
                sh.openTunnel();
            } else if (gamepad1.dpad_down) {
                sh.closeTunnel();
            }

            dashtele.addData("Velocity Shooter Lower", sh.getVelocityRPSLower());
            dashtele.addData("Velocity Shooter Upper", sh.getVelocityRPS());
            dashtele.addData("Velocity shooter", sh.getVelocityRPS());
            dashtele.addData("TARGET", sh.velocityTarget);
            dashtele.update();
        }

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

}