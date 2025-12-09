package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Goal 3 Artifacts", group = "1")
public class AutoRedGoal3Simple extends LinearOpMode {

    @Override
    public void runOpMode() {

        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting st = new Sorting(this);
        VoltageSensor sensor = hardwareMap.voltageSensor.iterator().next();

        Pose2d startPose = new Pose2d(-48, 50, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);
        st.wallForShooting();

        waitForStart();
        if (isStopRequested()) return;
        sh.continuousShooter.start();
        sh.setVelocityAuto(Shooter.VELO_GOAL);

        telemetry.addData("Power shooter", sh.shooter.getPower());


        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -5, Math.toRadians(-140)))
                .build());


        ///st.aprilTagToScan(ll.getTagID()); //сорян дшпш возможно бред написала
        sleep(1000);
        dt.turn(Math.toRadians(-50));
        sleep(1000);

//        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(dt.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(7, 27, Math.toRadians(90)))
//                .build());
        st.autoTurning();
        sh.continuousShooter.interrupt();
        telemetry.update();

    }


}