package org.firstinspires.ftc.teamcode.opmodes.test.drivetraintests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="TEST RR Custom", group = "Test")
public class DriveTrainRRTest extends LinearOpMode {
    private final DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
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
        if(isStopRequested()) return;
        dt.followTrajectorySequence(tr);

    }
}
