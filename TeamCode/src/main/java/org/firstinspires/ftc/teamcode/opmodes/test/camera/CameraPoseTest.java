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

@TeleOp(name = "TEST Pose Limelight", group="test")
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
            follower.startTeleopDrive();
            follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.left_trigger - gamepad1.right_trigger,
                            true // Robot Centric
                    );
            try {
                telemetry.addData("Tag ID", limelight3A.getTagInfo());
                telemetry.addData("X by Tag", limelight3A.getPoseByAprilTag().x);
                telemetry.addData("Y by Tag", limelight3A.getPoseByAprilTag().y);
                telemetry.addData("Z by Tag", limelight3A.getPoseByAprilTag().z);
                telemetry.addData("botpose", new Pose(limelight3A.getPoseByAprilTag().x,
                        limelight3A.getPoseByAprilTag().y,
                        limelight3A.getPoseByAprilTag().z,
                        FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));

                telemetry.update();
            } catch(Exception e){
                telemetry.addLine("Ахахахахаха лохи наллпойнтер");
            }
        }
        limelight3A.startOrStopLL(true);
    }
}
