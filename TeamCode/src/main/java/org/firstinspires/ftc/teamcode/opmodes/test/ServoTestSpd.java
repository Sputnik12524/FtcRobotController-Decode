package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ServoTestSpd extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo serv = hardwareMap.get(CRServo.class, "transferServo");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) serv.setPower(-1);
            if (gamepad1.bWasPressed()) serv.setPower(1);
            t.update();
        }
    }
}
