package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name="TeleOpRR", group="Tele")
public class TeleOpRoadRunner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

    }
    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}
