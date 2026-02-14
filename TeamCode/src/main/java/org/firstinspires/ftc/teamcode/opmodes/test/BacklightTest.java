package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Backlight;

@TeleOp(name="Backlight test", group="test")
@Config
public class BacklightTest extends LinearOpMode {
    Backlight blt;
    @Override
    public void runOpMode() throws InterruptedException {
        blt = new Backlight(this);
        waitForStart();
        if(gamepad1.aWasPressed()) blt.detectedGreen();
        else if (gamepad1.bWasPressed()) blt.detectedPurple();
        else if (gamepad1.xWasPressed()) blt.glowWhite();
        else blt.turnOffBacklight();
    }
}
