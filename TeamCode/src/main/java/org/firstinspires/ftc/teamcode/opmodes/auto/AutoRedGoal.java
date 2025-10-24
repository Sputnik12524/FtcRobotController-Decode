package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoRedGoal", group="Robot")
public class AutoRedGoal extends LinearOpMode {
    DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
    @Override
    public void runOpMode(){
        Pose2d startPose = new Pose2d(-47,-45, Math.toRadians(-45));
        dt.setPoseEstimate(startPose);

        waitForStart();

        if(isStopRequested()) return;

        dt.followTrajectorySequence(dt.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29,-25,Math.toRadians(-90)))
                .strafeTo(new Vector2d(-11,-27))
                .forward(5)
                .build());

    }
}
