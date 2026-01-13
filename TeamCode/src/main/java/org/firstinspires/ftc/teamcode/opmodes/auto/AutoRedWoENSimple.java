package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED WOEN Artifacts", group = "1")
public class AutoRedWoENSimple extends LinearOpMode {

    @Override
    public void runOpMode() {
        Intake in = new Intake(this);
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);


        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(180));
        dt.setPoseEstimate(startPose);

        sh.closeTunnel();
        sh.setLongThrowMode();

        waitForStart();
        if (isStopRequested()) return;

        in.rotateIn();
        sleep(1000);

        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW + 2);
        sh.continuousShooter.start();

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .back(10)
                .build());
        dt.turn(Math.toRadians(-22));
        sleep(1000);

        sleep(2000);

        sh.waitForShoot();
        sh.openTunnel();
        sleep(5000);

        sh.closeTunnel();
        in.rotateStop();
        sh.setVelocityTarget(0);
        dt.followTrajectory(dt.trajectoryBuilder(dt.getPoseEstimate())
                .forward(10)
                .build());
        dt.turn(Math.toRadians(-90));
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .back(28)
                .build());


        in.artifactIntake.interrupt();
        sh.continuousShooter.interrupt();

    }


}


