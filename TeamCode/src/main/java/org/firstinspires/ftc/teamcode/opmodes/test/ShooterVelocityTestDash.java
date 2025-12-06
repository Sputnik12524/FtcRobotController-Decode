package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Shooter;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "test dash shooter", group = "Test")
public class ShooterVelocityTestDash extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Shooter sh = new Shooter(this);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
        //        sh.shoot();
            } else {
                sh.shootStop();
            }

            dashTele.addData("Shooter Motor Velo", -sh.shooter.getVelocity()/1000);

            telemetry.addData("Shooter Motor Velocity:", -sh.shooter.getVelocity());

            dashTele.addData("power", sh.shooter.getPower());

            dashTele.update();

            telemetry.addData("Shooter Motor Power:", sh.shooter.getPower());
            telemetry.update();

        }
    }
}
