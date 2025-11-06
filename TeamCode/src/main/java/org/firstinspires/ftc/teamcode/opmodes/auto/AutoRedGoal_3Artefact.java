package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED 3 artefacts", group = "0")
public class AutoRedGoal_3Artefact extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(this.hardwareMap);
        Pose2d startPose = new Pose2d(34, 44, 45);
        dt.setPoseEstimate(startPose);

        waitForStart();
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(40,40,45))
                .build());
    }
}
