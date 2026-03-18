package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "2 MOTORS SHOOTER", group = "1")
@Config
@Disabled
public class Shooter2MotorsSeparateTest extends LinearOpMode {
    Shooter sh;
    Follower follower;

    /// Intake
    boolean isShootingShort = false;
    boolean isShootingLong = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean attentionControl = true;


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        sh = new Shooter(this);

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
            } else if (gamepad1.y && !stateY1 && isShootingLong){
                sh.closeTunnel();
                sh.shootStopUpper();
                isShootingLong = false;
            }
            stateY1 = gamepad1.y;


            if (gamepad1.x && !isShootingLong && !stateX1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocityLower();
                isShootingLong = true;
                isShootingShort = false;
            }else if (gamepad1.x && !stateX1 && isShootingLong) {
                sh.closeTunnel();
                sh.shootStopLower();
                isShootingLong = false;
            }
            stateX1 = gamepad1.x;

            if (gamepad1.dpad_up) {
                sh.openTunnel();
            } else if (gamepad1.dpad_down) {
                sh.closeTunnel();
            }


            telemetry.addData("Velocity shooter", sh.getVelocityRPS());
            telemetry.addData("TARGET", sh.velocityTarget);
            telemetry.update();
        }

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

}