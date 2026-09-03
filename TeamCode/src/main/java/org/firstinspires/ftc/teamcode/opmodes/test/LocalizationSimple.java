package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp

public class LocalizationSimple extends LinearOpMode {
    Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower.setStartingPose(new Pose(72,72,0));
        DriveTrain dt = new DriveTrain(this);

        waitForStart();

        while(opModeIsActive()){
            dt.setMotorsPowerNonLinear(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

            follower.update();
            t.addData("X", follower.getPose().getX() );
            t.addData("Y", follower.getPose().getY() );
            t.addData("Head", follower.getHeading() );
            t.update();

        }
    }
}
