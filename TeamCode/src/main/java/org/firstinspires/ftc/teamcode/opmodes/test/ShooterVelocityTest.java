package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Shooter", group="Test")
@Disabled
public class ShooterVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter sh = new Shooter(this);
        FileWriter writer = null;
        try {
            writer = new FileWriter("ShooterTest.txt", false); //false - файл перезаписывается при запуске программыит
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                sh.shoot();
            } else {
                sh.shootStop();
            }
            try {
                writer.write(Double.toString(sh.shooter.getPower()) + '\n');
            } catch (IOException e) {

            }
        }
        telemetry.addData("Shooter Motor Power:", sh.shooter.getPower());
        telemetry.update();
    }
}
