package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE Long 3 Artifacts", group = "1")
public class AutoBlueLong3Simple extends LinearOpMode {
    Follower follower;
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Intake in = new Intake(this);
        Shooter sh = new Shooter(this, follower);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);


        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(180));
        dt.setPoseEstimate(startPose);

        sh.closeTunnel();
        sh.setLongThrowMode();

        waitForStart();
        if (isStopRequested()) return;

        in.rotateIn();
        sleep(1000);

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .back(10)
                .build());
        dt.turn(Math.toRadians(22));
        sleep(1000);

        sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
        sh.continuousShooter.start();
        sleep(2000);

        sh.waitForShoot();
        sh.openTunnel();
        sleep(5000);

        sh.closeTunnel();
        in.rotateStop();
        sh.setVelocityTarget(0);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .back(15)
                .build());


        in.artifactIntake.interrupt();
        sh.continuousShooter.interrupt();

    }


}


