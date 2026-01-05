package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Human 3 Artifacts", group = "1")
public class AutoRedHuman3Simple extends LinearOpMode {

    @Override
    public void runOpMode() {

        Intake in = new Intake(this);
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(180));
        dt.setPoseEstimate(startPose);
        sh.closeCover();
        sh.setLongThrowMode();

        waitForStart();
        if (isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .back(10)
                .build());
        sleep(1000);
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
        sh.continuousShooter.start();
        in.artifactIntake.start();
        sleep(2000);
        sh.openCover();
        sleep(15000);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .back(15)
                .build());
        in.artifactIntake.interrupt();
        sh.continuousShooter.interrupt();

    }


}