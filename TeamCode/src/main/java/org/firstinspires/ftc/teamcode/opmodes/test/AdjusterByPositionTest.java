package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name="TEST Adjuster by Botpose", group="test")

public class AdjusterByPositionTest extends LinearOpMode {
    @Override
    public void runOpMode()  {
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,0);
        dt.setPoseEstimate(startPose);

        waitForStart();

        while(opModeIsActive()){
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

            dt.update();
            Pose2d current = dt.getPoseEstimate();
            if(current.getX() >= 48) {
                sh.openCover();
            } else {
                sh.closeCover();
            }

        }

    }

}
