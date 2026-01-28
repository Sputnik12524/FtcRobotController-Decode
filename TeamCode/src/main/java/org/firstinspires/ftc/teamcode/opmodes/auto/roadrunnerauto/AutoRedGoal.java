package org.firstinspires.ftc.teamcode.opmodes.auto.roadrunnerauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
@Disabled
@Autonomous(name = "RED Goal 3+3 Artifacts", group = "AutoRed")
public class AutoRedGoal extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

        Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, 25))
        //scoring
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-11, 27, Math.toRadians(90)))
                .forward(20)
        //capturing
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-29, 25, Math.toRadians(-45)))
                .build());
        //scoring
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-50, 25, Math.toRadians(0)))
                .build());
    }
}
