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

@TeleOp(name = "TEST Shooter, Sorting and their friends!", group = "1")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
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
       // logger = new Logger("ShVelocityTime");
        sh = new Shooter(this);
       // ll = new Limelight(this);
        in = new Intake(this);


        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        MotorConfigurationType motorConfigurationType = sh.shooterUpper.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        sh.shooterUpper.setMotorType(motorConfigurationType);

        timer.reset();
       // logger.addHeader("Time,Velocity");

        waitForStart();
        while (opModeIsActive()) {

            // SHOOTER
            if (gamepad1.y && !stateY1 && !isShooting) {
                sh.setVelocityTarget(RPS);
                sh.shootByVelocity();
                isShooting = true;
            } else if (gamepad1.y && !stateY1 && isShooting) {
                sh.shootStop();
                isShooting = false;
            }
            stateY1 = gamepad1.y;

            // TURRET

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


            // ADJUSTER
            if (gamepad1.dpad_up && POS_ADJUSTER <= 1) {
                POS_ADJUSTER += 0.005;
                sh.angleAdjuster.setPosition(POS_ADJUSTER);
            } else if (gamepad1.dpad_down && POS_ADJUSTER >= 0) {
                POS_ADJUSTER -= 0.005;
                sh.angleAdjuster.setPosition(POS_ADJUSTER);
            }
            if (gamepad1.dpad_left) {
                sh.openTunnel();
            } else if (gamepad1.dpad_right) {
                sh.closeTunnel();
            }

            // LOGGER
          //  logger.addLine(timer.milliseconds(), sh.getVelocityRPS());

            telemetry.addData("real PRS", sh.getVelocityRPS());
            telemetry.addData("real TPS", sh.getVelocityTPS());
            telemetry.addData("ADJUSTER POS", sh.getAngleAdjusterPos());
            telemetry.update();

            dashTele.addLine("SHOOTER:");
            dashTele.addData("target of RPS:", RPS);
            dashTele.addData("real RPS:", sh.getVelocityRPS());
            dashTele.addData("real RPS LOWER:", sh.shooterLower.getVelocity());
            dashTele.addData("real TPS HIGHER:", sh.shooterUpper.getVelocity());
            dashTele.addData("Value of encoders:", sh.shooterUpper.getCurrentPosition());
            dashTele.addData("ADJUSTER POS", POS_ADJUSTER);
            dashTele.update();
        }
       // logger.fileClose();
    }
}


