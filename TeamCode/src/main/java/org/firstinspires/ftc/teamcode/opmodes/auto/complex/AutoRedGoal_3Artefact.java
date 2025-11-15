package org.firstinspires.ftc.teamcode.opmodes.auto.complex;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED 3 artefacts", group = "0")
public class AutoRedGoal_3Artefact extends LinearOpMode {
    Sorting sorting;
    Shooter shooter;
    ElapsedTime timer;
    Limelight limeLight;

    @Override
    public void runOpMode() {
        limeLight = new Limelight(this);
        timer = new ElapsedTime();
        shooter = new Shooter(this);
        sorting = new Sorting(this, limeLight);
        DriveTrainMecanum dt = new DriveTrainMecanum(this.hardwareMap);
        Pose2d startPose = new Pose2d(-48, 50, 130);
        dt.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        sorting.wallForShooting();
        shooter.continuousShooter.start();
        shooter.setPower(0.8);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 0, 110))
                .waitSeconds(3)
                .build());
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, 0, 65))
                .build());
        sorting.autoTurning();
        shooter.continuousShooter.interrupt();
    }
}