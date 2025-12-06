package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "BLUE Goal 3 Artifacts", group = "0")
public class AutoBlueGoal3 extends LinearOpMode {


    @Override
    public void runOpMode() {
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting st = new Sorting(this);
        Thread main = Thread.currentThread();


        Pose2d startPose = new Pose2d(-48, -50, Math.toRadians(45));
        dt.setPoseEstimate(startPose);
        st.wallForShooting();
        st.regulatorSorting.start();

        waitForStart();
        if (isStopRequested()) return;
        sh.continuousShooter.start();

        sh.setVelocityAuto(0.6);

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, 5, Math.toRadians(140)))
                .build());

        ll.getTagID();

        st.sortingArtefacts(st.artefact_pos(st.getColor()), st.aprilTagToScan(ll.tagId));
        try {
            main.join(1500);
        } catch (InterruptedException e) {
            st.exception = String.valueOf(e);
        }

        sleep(1000);
        dt.turn(Math.toRadians(53));
        sleep(1000);

        st.sortShooter.start();

        try {
            main.join();
        } catch (InterruptedException e) {
            st.exception = String.valueOf(e);
        }

        sh.continuousShooter.interrupt();
        st.sortShooter.interrupt();
        st.regulatorSorting.interrupt();

    }
}
