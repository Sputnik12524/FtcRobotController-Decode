package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Goal 3 Artifacts", group = "1")
public class AutoBlueGoal3Simple extends LinearOpMode {


    @Override
    public void runOpMode() {
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Intake in = new Intake(this);

        Pose2d startPose = new Pose2d(-48, -50, Math.toRadians(45));
        dt.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
        sh.continuousShooter.start();

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .forward(18)
                //.lineToLinearHeading(new Pose2d(-27, 5, Math.toRadians(140)))
                .build());
        sleep(1000);
        sh.openCover();
        in.artifactIntake.start();
        sleep(10000);

        in.artifactIntake.interrupt();
        sh.continuousShooter.interrupt();

    }
}