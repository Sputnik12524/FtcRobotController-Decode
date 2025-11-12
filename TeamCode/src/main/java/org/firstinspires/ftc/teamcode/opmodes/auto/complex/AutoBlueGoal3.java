package org.firstinspires.ftc.teamcode.opmodes.auto.complex;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Goal 3 Artifacts", group = "AutoBlue")
public class AutoBlueGoal3 extends LinearOpMode {


    @Override
    public void runOpMode() {
        DriveTrainMecanum dt = new DriveTrainMecanum(this.hardwareMap);
        Limelight ll = new Limelight(this);
        Shooter sh = new Shooter(this);
        Sorting sort = new Sorting(this);

        Pose2d startPose = new Pose2d(-49, -49, Math.toRadians(45));

        dt.setPoseEstimate(startPose);
        ll.startOrStopLL(false);

        waitForStart();

        if (isStopRequested()) return;


        sh.continuousShooter.start();

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29, -25, Math.toRadians(170)))
                .build());

        sort.aprilTagToScan(ll.getTagID()); //сорян дшпш возможно бред написала
        sleep(1000);
        dt.turn(Math.toRadians(45));
        sleep(1000);

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-7, -27, Math.toRadians(90)))
                .build());
        ll.startOrStopLL(true);
        sh.continuousShooter.interrupt();
    }

}
