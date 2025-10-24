package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class DriveTrainRRTest extends LinearOpMode {
    private DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
    @Override
    public void runOpMode(){
        Pose2d startPose = new Pose2d(0,0,0);
        dt.setPoseEstimate(startPose);
        TrajectorySequence tr = dt.trajectorySequenceBuilder(startPose)
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        dt.followTrajectorySequence(tr);

    }
}
