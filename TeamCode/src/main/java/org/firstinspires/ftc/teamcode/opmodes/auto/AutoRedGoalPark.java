package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Goal Park", group = "RedPark")
public class AutoRedGoalPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-55, 14, Math.toRadians(90)))
                .build());

    }
}
