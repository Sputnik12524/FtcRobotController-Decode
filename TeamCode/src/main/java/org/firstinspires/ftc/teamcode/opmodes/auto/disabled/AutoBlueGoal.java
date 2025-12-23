package org.firstinspires.ftc.teamcode.opmodes.auto.disabled;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Goal 3+3 Artifact", group = "AutoBlue")

public class AutoBlueGoal extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

        Pose2d startPose = new Pose2d(-48, -50, Math.toRadians(45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, -25))
                        .turn(Math.toRadians(-180))
                .build());
        //scoring
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(-5, -24, Math.toRadians(-90)))
                .waitSeconds(5)
                .forward(20)
                //capturing
                .build());
/*
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-29, -25, Math.toRadians(45)))
        //scoring
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-50, -25, Math.toRadians(0))) //that's parking
                .build());*/

    }
}
