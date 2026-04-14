package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ControlHubTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        telemetry.addLine("I'  in init");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addLine("i'm okay!");
            telemetry.update();
        }

    }
}
