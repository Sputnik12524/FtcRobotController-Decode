package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.util.Logger;

@Config
@TeleOp(name = "TEST Shooter/Intake/Adjuster/Cover", group = "1")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Limelight ll;
    Logger logger;
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
        logger = new Logger("ShVelocityTime");
        sh = new Shooter(this);
        ll = new Limelight(this);
        in = new Intake(this);


        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        MotorConfigurationType motorConfigurationType = sh.shooterUpper.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        sh.shooterUpper.setMotorType(motorConfigurationType);

        timer.reset();
        logger.addHeader("Time,Velocity");

        waitForStart();
        while (opModeIsActive()) {
            sh.threeArtefactsShooting();


            // LOGGER
            logger.addLine(timer.milliseconds(), sh.getVelocityRPS());

//            telemetry.addData("real PRS", sh.getVelocityRPS());
//            telemetry.addData("real TPS", sh.getVelocityTPS());
            telemetry.addData("ADJUSTER POS", sh.getAngleAdjusterPos());
            telemetry.addData("Засекли выстрел", sh.detected);
            telemetry.update();

            dashTele.addLine("SHOOTER:");
            dashTele.addData("target of RPS:", RPS);
            dashTele.addData("real RPS:", sh.getVelocityRPS());
            dashTele.addData("real TPS:", sh.getVelocityTPS());
            dashTele.addData("Value of encoders:", sh.shooterUpper.getCurrentPosition());
            dashTele.addData("ADJUSTER POS", POS_ADJUSTER);
            dashTele.update();
        }
        logger.fileClose();
    }
}


