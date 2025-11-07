package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Shooter;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Shooter", group = "Test")
public class ShooterVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Shooter sh = new Shooter(this);


////        FileWriter writer = null;
//        try {
//            writer = new FileWriter("ShooterTest.txt", false); //false - файл перезаписывается при запуске программыит
//        } catch (IOException e) {
//            throw new RuntimeException(e);
     //   }
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.a) {
                sh.shoot();
            } else {
                sh.shootStop();
            }
//            try {
//           //     writer.write(timer.milliseconds() + ' ' + Double.toString(sh.shooter.getVelocity()) + '\n');
//            } catch (IOException e)
//            {

     //       }
            dashTele.addData("Shooter Motor Velo in degrees", sh.shooter.getVelocity());

            telemetry.addData("Shooter Motor Velocity:", sh.shooter.getVelocity());

            dashTele.addData("power", sh.shooter.getPower());

            dashTele.update();

            telemetry.addData("Shooter Motor Power:", sh.shooter.getPower());
            telemetry.update();
        }
    }
}
