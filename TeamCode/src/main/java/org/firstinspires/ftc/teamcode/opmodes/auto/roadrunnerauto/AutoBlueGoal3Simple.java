package org.firstinspires.ftc.teamcode.opmodes.auto.roadrunnerauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE Goal 3 Artifacts", group = "1")
@Disabled
public class AutoBlueGoal3Simple extends LinearOpMode {

    Follower follower;
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Shooter sh = new Shooter(this, follower);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Intake in = new Intake(this);

        Pose2d startPose = new Pose2d(-48, -50, Math.toRadians(45));
        dt.setPoseEstimate(startPose);

        sh.closeTunnel();
        sh.setShortThrowMode();

        waitForStart();
        if (isStopRequested()) return;

        in.rotateIn();
        sleep(1000);

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose).forward(25).build());

        sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW - 5);
        sleep(2000);

        sh.openTunnel();
        sleep(5000);

        sh.closeTunnel();
        in.rotateStop();
        sh.setVelocityTarget(0);


        dt.turn(Math.toRadians(90));
           dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate()).forward(18).build());

        in.artifactIntake.interrupt();

    }
}