package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Human 3 Artifact", group = "AutoRed")
public class AutoRedHuman extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        //съезд с красной зоны к артефактам
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, 30, Math.toRadians(90)))
                .build());

    }
}
