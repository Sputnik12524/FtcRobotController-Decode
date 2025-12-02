package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@Config
@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Sorting sr;
    Limelight ll;

    public static double RPS = 10;

    @Override
    public void runOpMode() {
        sh = new Shooter(this);
        ll = new Limelight(this);
        in = new Intake(this);
        sr = new Sorting(this);

        sr.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sh.shooterUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sh.shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            FtcDashboard dash = FtcDashboard.getInstance();
            Telemetry dashTele = dash.getTelemetry();
            sh.setVelocityUp(RPS);


            dashTele.addData("ticks per second", sh.shooterUp.getVelocity());
            dashTele.addData("degrees per second", sh.shooterUp.getVelocity(AngleUnit.DEGREES));
            dashTele.addData("radians per second", sh.shooterUp.getVelocity(AngleUnit.RADIANS));
            dashTele.addData("VELOCITY", (sh.shooterUp.getVelocity() / 28));
            dashTele.addData("ENCODERS", sh.shooterUp.getCurrentPosition());

            dashTele.update();
        }
    }
}


