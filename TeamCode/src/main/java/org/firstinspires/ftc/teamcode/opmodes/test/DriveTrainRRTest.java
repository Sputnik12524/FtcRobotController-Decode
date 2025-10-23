package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

public class DriveTrainRRTest extends LinearOpMode {
    private DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
    @Override
    public void runOpMode(){
        dt.setPoseEstimate(new Pose2d(0,0));
    }
}
