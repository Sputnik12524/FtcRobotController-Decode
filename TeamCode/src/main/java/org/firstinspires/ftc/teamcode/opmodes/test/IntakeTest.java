package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;

@Config

@TeleOp(name = "Transfer and intake yayica test", group = "1")
public class IntakeTest extends LinearOpMode {
    Intake in;
   ElapsedTime timer;

    public static double RPS = 25;

    public double POS_ADJUSTER = 0.5;

    boolean stateA1 = false;
    boolean stateB1 = false;
    boolean stateY1 = false;
    boolean isRotateIn = false;
    boolean isRotateOut = false;
    boolean isShooting = false;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        in = new Intake(this);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        timer.reset();
       // logger.addHeader("Time,Velocity");

        waitForStart();
        while (opModeIsActive()) {
            // SHOOTER :(
            // TURRET :(

            // INTAKE
            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
            }
            if (gamepad1.b && !isRotateOut && !stateB1) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
            } else if (gamepad1.b && isRotateOut && !stateB1) {
                in.rotateStop();
                isRotateOut = false;
            }
            stateA1 = gamepad1.a;
            stateB1 = gamepad1.b;

            telemetry.update();

            dashTele.addLine("SHOOTER:");
            dashTele.addData("target of RPS:", RPS);
            dashTele.addData("ADJUSTER POS", POS_ADJUSTER);
            dashTele.update();
        }
       // logger.fileClose();
    }
}


