package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "0 Park", group = "Park")
public class AutoPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .forward(20)
                .build());

    }
}
