package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Backlight;
import org.firstinspires.ftc.teamcode.modules.Transfer;

@TeleOp(name="Backlight test", group="test")
@Config
public class BacklightTest extends LinearOpMode {
    Backlight blt;
    Transfer transfer;
    @Override
    public void runOpMode() throws InterruptedException {
        blt = new Backlight(this, transfer);
        blt.lights();
        waitForStart();

    }
}
