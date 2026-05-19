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

    public boolean isOutput = false;
    public boolean stateX1 = false;


    @Override
    public void runOpMode() {
        DigitalChannel led1 = hardwareMap.get(DigitalChannel.class,"LED1");
        DigitalChannel led2 = hardwareMap.get(DigitalChannel.class, "LED2");
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                led1.setState(true);
                led2.setState(true);
                telemetry.addData("State of green", led1.getState());
                telemetry.addData("State of purple", led2.getState());
            } else {
                led1.setState(false);
                led2.setState(false);
                telemetry.addData("State of green", led1.getState());
                telemetry.addData("State of purple", led2.getState());
            }
            if (gamepad1.x && !isOutput && !stateX1) {
                led1.setMode(DigitalChannel.Mode.INPUT);
                led2.setMode(DigitalChannel.Mode.INPUT);
            } else if (gamepad1.x && !stateX1 && isOutput) {
                led1.setMode(DigitalChannel.Mode.OUTPUT);
                led2.setMode(DigitalChannel.Mode.OUTPUT);
            }
            telemetry.addData("Mode of green", led1.getMode());
            telemetry.addData("Mode of purple", led2.getMode());
            telemetry.update();
        }
    }

}
