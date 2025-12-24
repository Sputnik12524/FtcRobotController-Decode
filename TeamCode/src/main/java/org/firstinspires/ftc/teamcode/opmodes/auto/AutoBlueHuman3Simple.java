package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Human 3 Artifacts", group = "1")
public class AutoBlueHuman3Simple extends LinearOpMode {

    @Override
    public void runOpMode() {

        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting st = new Sorting(this);

        Pose2d startPose = new Pose2d(62, -10, Math.toRadians(180));
        dt.setPoseEstimate(startPose);
        st.wallForShooting();

        waitForStart();
        if (isStopRequested()) return;
        sleep(10000);
        sh.continuousShooter.start();
        sh.shootByVelocity(Shooter.VELOCITY_FOR_LONG_THROW);

        sleep(1000);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(59, -10, Math.toRadians(200)))
                .build());
        sleep(3000);

        st.autoTurning();
        sh.continuousShooter.interrupt();
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(180)))
                .build());


    }


}