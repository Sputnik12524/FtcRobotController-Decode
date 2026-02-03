package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
@TeleOp(name = "TEST Turret", group = "1")
public class TurretTest extends LinearOpMode {

    Follower follower;

    public static double TARGET = 0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();



        waitForStart();
        while (opModeIsActive()) {
            follower.update();
//            if (gamepad1.right_bumper) {
//                tr.turnRightByTarget();
//            } else if (gamepad1.left_bumper) {
//                tr.turnLeftByTarget();
//            }

//
        }
    }
}
