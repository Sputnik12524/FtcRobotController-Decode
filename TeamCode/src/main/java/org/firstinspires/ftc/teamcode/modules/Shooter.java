package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {//sh
    public final DcMotorEx shooterUpper;
    public final DcMotorEx shooterLower;
    private final Servo angleAdjuster;
    private final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(30, 0, 30, 27);
    private final double TPR = 28;
    public volatile int artifacts = 0;
    public int timers;
    public static double POWER = 1;
    public static double VELOCITY = 0;
    public static double VELOCITY_FOR_LONG_THROW = 45;
    public static double VELOCITY_FOR_SHORT_THROW = 37;
    public static double ERROR = 3;
    public static double POS_COVER_OPEN = 0;
    public static double POS_COVER_CLOSE = 1;
    public static double POS_SHORT_THROW = 0;
    public static double POS_LONG_THROW = 1;
    boolean isShooting = false;

    public ContinuousShooter continuousShooter = new ContinuousShooter();

    public Shooter(LinearOpMode opMode) {
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");

        shooterUpper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterUpper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterLower.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(shooterUpper, MOTOR_VELO_PID_SHOOTERS);
        setPIDFCoefficients(shooterLower, MOTOR_VELO_PID_SHOOTERS);
    }

    public void shootByVelocity(double RPS) {
        shooterUpper.setVelocity(RPS * TPR);
        shooterLower.setVelocity(RPS * TPR);
    }

    public void shootByPower(double POWER) {
        shooterUpper.setPower(POWER);
        shooterLower.setPower(POWER);
    }

    public void shootStop() {
        shooterUpper.setVelocity(0);
        shooterLower.setVelocity(0);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void waitForShoot(double velocity) {
        shootByVelocity(velocity);
        while (getVelocityRPS() >= VELOCITY_FOR_SHORT_THROW + ERROR || getVelocityRPS() <= VELOCITY_FOR_SHORT_THROW - ERROR);
        //тут открытие крышки
    }

    public void setShortThrowMode() {
        angleAdjuster.setPosition(POS_SHORT_THROW);
    }
    public void setLongThrowMode() {
        angleAdjuster.setPosition(POS_LONG_THROW);
    }
    public void openCover() {
        cover.setPosition(POS_COVER_OPEN);
    }
    public void closeCover() {
        cover.setPosition(POS_COVER_CLOSE);
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                shooterUpper.setVelocity(VELOCITY);
                while (timer.milliseconds() < 23000);
                shooterUpper.setVelocity(0);
            }
        }
    }

    public void setVelocityAuto(double RPS) { //For ContinuousShooter
        VELOCITY = RPS * TPR;
    }

    public void updateCalculator() {
        if (shooterUpper.getVelocity() < VELOCITY) {
            artifacts++;
            timers = 1000;
        }
        timers = 50;
    }

    public double getPower() {
        return shooterUpper.getPower();
    }
    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }
    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }


}
