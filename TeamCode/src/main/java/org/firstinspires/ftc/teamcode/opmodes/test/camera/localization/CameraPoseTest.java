package org.firstinspires.ftc.teamcode.opmodes.test.camera.localization;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TEST Pose Limelight", group = "test")
@Config
public class CameraPoseTest extends LinearOpMode {
    Limelight limelight3A;
    Follower follower;


    @Override
    public void runOpMode() {
        Turret tt = new Turret(this);
        follower = Constants.createFollower(hardwareMap);

        limelight3A = new Limelight(this, tt, follower);
        follower.setStartingPose(new Pose(72, 72, 0));

        limelight3A.startOrStopLL(false);
        waitForStart();

        while (opModeIsActive()) {
            limelight3A.update();

            follower.update();

            telemetry.addData("Tag ID", limelight3A.getTagInfo()[0]);
            telemetry.addData("botpose", limelight3A.getRawPose()[0]);
            telemetry.addData("botpose", limelight3A.getRawPose()[1]);
            telemetry.addData("botpose", limelight3A.getRawPose()[2]);//2,1,0 если не переводить, 1.000000000000002, -2, 540 еслт перевести в педро
            // limelight3A.relocalizeWhenError();

            if (gamepad1.b) {
                telemetry.addData("Tag ID", limelight3A.getTagInfo()[0]);
                telemetry.addData("X by Tag", limelight3A.getGoalTag()[2]);
                telemetry.addData("Y by Tag", limelight3A.getGoalTag()[3]);
            }


            telemetry.update();
        }
        limelight3A.startOrStopLL(true);
    }
}
