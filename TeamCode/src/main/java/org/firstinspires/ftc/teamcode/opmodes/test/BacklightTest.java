package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Backlight;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

@TeleOp(name="Backlight test", group="test")
@Config
@Disabled
public class BacklightTest extends LinearOpMode {
    Backlight bl;
    GamepadManager g1;
    GamepadManager g2;
    @Override
    public void runOpMode() throws InterruptedException {
        bl = new Backlight(this, new Transfer(this));
        g1 = new GamepadManager(gamepad1);
        g2 = new GamepadManager(gamepad2);

        waitForStart();

        while(opModeIsActive()){
            g1.update();
            g2.update();
            if(g1.A.isPressed() && g1.A.getToggleState()){
                bl.G1.setState(true);
                bl.P1.setState(false);
            }
            else if(g1.A.isPressed() && !g1.A.getToggleState()){
                bl.G1.setState(false);
                bl.P1.setState(true);
            }
            if(g1.B.isPressed() && g1.B.getToggleState()){
                bl.G1.setState(true);
                bl.P1.setState(true);
            }
            else if(g1.B.isPressed() && !g1.B.getToggleState()){
                bl.G1.setState(false);
                bl.P1.setState(false);
            }
        }

    }
}
