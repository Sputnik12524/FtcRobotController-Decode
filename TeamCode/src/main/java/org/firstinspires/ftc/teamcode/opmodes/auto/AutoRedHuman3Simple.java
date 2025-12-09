package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Human 3 Artifacts", group = "1")
public class AutoRedHuman3Simple extends LinearOpMode {

    @Override
    public void runOpMode() {

        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting st = new Sorting(this);

        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(180));
        dt.setPoseEstimate(startPose);
        st.wallForShooting();

        waitForStart();
        if (isStopRequested()) return;
        sh.continuousShooter.start();
        sh.shootByVelocity(Shooter.VELO_HUMAN);


        ///st.aprilTagToScan(ll.getTagID()); //сорян дшпш возможно бред написала
        sleep(1000);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(59, 10, Math.toRadians(-180)))
                .build());
        sleep(3000);

//        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(7, 27, Math.toRadians(90)))
//                .build());
        st.autoTurning();
        sh.continuousShooter.interrupt();

    }


}