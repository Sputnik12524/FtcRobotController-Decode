package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;

@TeleOp(name = "TEST Auto Limelight")
public class CameraAutoMethodTest extends LinearOpMode {
    Limelight limelight3A;

    @Override
    public void runOpMode() {
        limelight3A = new Limelight(this);

        limelight3A.startOrStopLL(false);
        waitForStart();

        while (opModeIsActive()) {
           telemetry.addData("Tag ID", limelight3A.getTagID());
           telemetry.update();
        }
        limelight3A.startOrStopLL(true);
    }
}
