package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Human 3 Artifact", group = "AutoBlue")
public class AutoBlueHuman extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(62, -10, Math.toRadians(180));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        //съезд с линии синий зоны к артефактам.
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(-90)))
                .build());

    }
}
