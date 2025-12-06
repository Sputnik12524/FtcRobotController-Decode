package org.firstinspires.ftc.teamcode.opmodes.test;

import android.os.Environment;

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
import java.io.File;

import util.Logger;

@TeleOp(name = "Shooter", group = "Test")
public class ShooterVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Logger logger = new Logger("Shooter_AllTele.csv");
        String directoryPath = "/sdcard/FIRST/";
        ElapsedTime timer = new ElapsedTime();
        Shooter sh = new Shooter(this);


        logger.addHeader("time_ms,velocity");

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
           //     sh.shoot();
            } else {
                sh.shootStop();
            }

            logger.addLine(timer.milliseconds(), -sh.shooter.getVelocity() / 28);

            dashTele.addData("Shooter Motor Velo in degrees", -sh.shooter.getVelocity());

            telemetry.addData("Shooter Motor Velocity:", -sh.shooter.getVelocity());

            dashTele.addData("power", sh.shooter.getPower());

            dashTele.update();

            telemetry.addData("Shooter Motor Power:", sh.shooter.getPower());
            telemetry.update();

        }
        logger.fileClose();
    }
}
