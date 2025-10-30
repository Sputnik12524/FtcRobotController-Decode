package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Goal 3 Artifacts", group = "AutoRed")
@Disabled
public class AutoRedGoal3 extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, 25))
        //scoring
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-7, 27, Math.toRadians(90)))
                .build());

    }
}
