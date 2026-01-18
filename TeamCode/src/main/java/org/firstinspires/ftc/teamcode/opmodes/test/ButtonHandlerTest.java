package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

@Config
@TeleOp(name = "ButtonHandler", group = "Test")
public class ButtonHandlerTest extends LinearOpMode {
    GamepadManager g1;
    GamepadManager g2;

    Telemetry t = new MultipleTelemetry(telemetry);
    public void runOpMode() {
        g1 = new GamepadManager(gamepad1);
        g2 = new GamepadManager(gamepad2);

        waitForStart();
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            if(g1.A.isPressed()){
                if(g1.A.getToggleState()) t.addLine("Нажали А");
                else t.addLine(" Еще раз А");
            }
            if(g1.A.isReleased()) t.addLine("А отпущена");

            if(g1.B.isHeld()) t.addLine("B зажата");
            if(g1.X.isHeldFor(2500)) t.addLine("Х зажата 2,5 секунды");
            telemetry.update();
        }
    }
}
