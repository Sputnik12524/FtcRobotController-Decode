package org.firstinspires.ftc.teamcode.opmodes.auto.complex;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Goal 3+3 Artifact", group = "AutoBlue")
@Disabled
public class AutoBlueGoal extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(62, -10, Math.toRadians(45));
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
        //capturing
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
