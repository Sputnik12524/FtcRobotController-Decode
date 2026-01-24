package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    public static double POSITION = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo serv = hardwareMap.get(Servo.class, "angleAdjuster");

        waitForStart();
        resetRuntime();

        while (opModeIsActive()){
            serv.setPosition(POSITION);
            if(gamepad1.a) POSITION += 0.001;
            if(gamepad1.b) POSITION -= 0.001;
        }
    }
}
