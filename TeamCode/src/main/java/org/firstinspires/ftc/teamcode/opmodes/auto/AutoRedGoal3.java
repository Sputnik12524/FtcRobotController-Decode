package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Goal 3 Artifacts", group = "0")
public class AutoRedGoal3 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting sort = new Sorting(this);

        Pose2d startPose = new Pose2d(-48, 50, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);
        sort.wallForShooting();

        waitForStart();
        if (isStopRequested()) return;
        sh.continuousShooter.start();
        sh.setPower(0.76);

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -5, Math.toRadians(-140)))
                .build());


        ///sort.aprilTagToScan(ll.getTagID()); //сорян дшпш возможно бред написала
        sleep(1000);
        dt.turn(Math.toRadians(-65));
        sleep(1000);

//        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(7, 27, Math.toRadians(90)))
//                .build());
        sort.autoTurning();
        sh.continuousShooter.interrupt();

    }


}