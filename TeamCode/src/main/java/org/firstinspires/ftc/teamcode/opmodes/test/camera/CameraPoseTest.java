package org.firstinspires.ftc.teamcode.opmodes.test.camera;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TEST Pose Limelight", group = "test")
@Config
public class CameraPoseTest extends LinearOpMode {
    Limelight limelight3A;
    Follower follower;

    @Override
    public void runOpMode() {
        limelight3A = new Limelight(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0));

        limelight3A.startOrStopLL(false);
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            try {
                telemetry.addData("Tag ID", limelight3A.getTagInfo());
                telemetry.addData("botpose", limelight3A.getPoseByAprilTag());
               // limelight3A.relocalizeWhenError();

                if (gamepad1.b) {
                    telemetry.addData("Tag ID", limelight3A.getTagInfo());
                    telemetry.addData("X by Tag", limelight3A.getGoalTag().get(2));
                    telemetry.addData("Y by Tag", limelight3A.getGoalTag().get(3));
                }

            } catch (Exception e) {
                telemetry.addLine("Ахахахахаха лохи наллпойнтер");
            }
            telemetry.update();
        }
        limelight3A.startOrStopLL(true);
    }
}
