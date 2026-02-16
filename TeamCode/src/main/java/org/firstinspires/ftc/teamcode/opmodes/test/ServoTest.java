package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    public static double POSITION = 0.0;
    public static double DIFF = 0.0005;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo serv = hardwareMap.get(Servo.class, "angleAdjuster");

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            serv.setPosition(POSITION);
            if (gamepad1.aWasPressed()) POSITION += DIFF;
            if (gamepad1.bWasPressed()) POSITION -= DIFF;


            t.addData("Potuzhnaya stenka", serv.getPosition());
            t.addData("DIFF", DIFF);
            t.update();
        }
    }
}
