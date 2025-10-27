package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "AutoBlueGoal", group = "Robot")
public class AutoBlueGoal extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-47, -45, Math.toRadians(45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, -25))
                .build());
        //scoring
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-11, -27, Math.toRadians(-90)))
                .forward(20)
        //intaking
                .build());

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-29, -25, Math.toRadians(45)))
        //scoring
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-50, -25, Math.toRadians(0))) //that's parking
                .build());

    }
}
