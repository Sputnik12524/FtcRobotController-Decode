package org.firstinspires.ftc.teamcode.opmodes.auto.complex;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED 3 artefacts", group = "0")
public class AutoRedGoal_3Artefact extends LinearOpMode {
    Sorting sorting;
    Shooter shooter;
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        shooter = new Shooter(this);
        sorting = new Sorting(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(this.hardwareMap);
        Pose2d startPose = new Pose2d(-48, 50, 130);
        dt.setPoseEstimate(startPose);
        sorting.dwallOpen();
        sorting.hwallClose();

        waitForStart();
        if (isStopRequested()) return;
        shooter.continuousShooter.start();
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 0, 180))
                .waitSeconds(3)
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-56, -22, 90))
                .build());
        sorting.autoDrumTurningIn(0.4);
        shooter.continuousShooter.interrupt();
    }
}