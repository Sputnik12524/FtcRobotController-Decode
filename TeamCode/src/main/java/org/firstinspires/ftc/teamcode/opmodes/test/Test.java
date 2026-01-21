package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Shooter;

@Config
@TeleOp
public class Test extends LinearOpMode {
    public static double POSITION = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter sh = new Shooter(this);
        Telemetry t = new MultipleTelemetry(telemetry);


        waitForStart();
        resetRuntime();

        while (opModeIsActive()){
            t.addData("Робот: ", sh.isEmpty());
        }
    }
}

