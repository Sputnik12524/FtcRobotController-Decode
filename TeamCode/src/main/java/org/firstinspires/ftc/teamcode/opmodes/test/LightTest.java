package org.firstinspires.ftc.teamcode.opmodes.test;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class LightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DigitalChannel led = hardwareMap.get(DigitalChannel.class,"LED1");
        led.setMode(DigitalChannel.Mode.OUTPUT);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                led.setState(true);
                telemetry.addData("State of sensor", led.getState());
            } else {
                led.setState(false);
                telemetry.addData("State of sensor", led.getState());
            }
            telemetry.addData("Mode of sensor", led.getMode());
            telemetry.update();
        }
    }

}
