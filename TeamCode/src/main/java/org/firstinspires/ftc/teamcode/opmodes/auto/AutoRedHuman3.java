package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@Autonomous(name = "RED Human 3 Artifacts", group = "0")
public class AutoRedHuman3 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        Limelight ll = new Limelight(this);
        Sorting st = new Sorting(this);
        Thread main = Thread.currentThread();

        Pose2d startPose = new Pose2d(61, 7, Math.toRadians(180));
        dt.setPoseEstimate(startPose);
        st.wallForShooting();


        waitForStart();
        if (isStopRequested()) return;
        sh.continuousShooter.start();
        st.regulatorSorting.start();
        sh.setPower(1);


        ///st.aprilTagToScan(ll.getTagID()); //сорян дшпш возможно бред написала
        st.sortingArtefacts(st.artefact_pos(st.getColor()), st.aprilTagToScan(ll.tagId));
        try {
            main.join(1500);
        } catch (InterruptedException e) {
            st.exception = String.valueOf(e);
        }

        sleep(1000);
        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(59, 10, Math.toRadians(-180)))
                .build());
        sleep(3000);

        st.sortShooter.start();

        try {
            main.join();
        } catch (InterruptedException e) {
            st.exception = String.valueOf(e);
        }

        st.autoTurning();
        sh.continuousShooter.interrupt();
        st.regulatorSorting.interrupt();
        st.sortShooter.interrupt();

    }


}