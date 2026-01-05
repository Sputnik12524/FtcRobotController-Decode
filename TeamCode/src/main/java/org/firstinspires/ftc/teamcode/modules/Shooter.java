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
    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(20, 0, 30, 14.7);
    private final double TPR = 28;
    public int artifacts = 0;
    public int artifactsNow = 0;
    public int timers;
    public static double POWER = 1;
    public double velocityTarget = 44;
    public static double VELOCITY_FOR_LONG_THROW = 50;
    public static double VELOCITY_FOR_SHORT_THROW = 43;
    public static double VELOCITY_ZERO = 0;
    public static double ERROR = 3;
    public static double POS_COVER_OPEN = 0.72;
    public static double POS_COVER_CLOSE = 1;
    public static double POS_SHORT_THROW = 0.75;
    public static double POS_LONG_THROW = 1;
    public boolean canShoot = false;
    boolean needShootPortion = false;

    public static long TIME_GATES_BETWEEN_SHOOT = 1000;
    public static long TIME_FOR_SET_VELOCITY = 2500;


    public ContinuousShooter continuousShooter = new ContinuousShooter();
    public ShooterPortion portion = new ShooterPortion();

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

    public void shootByVelocity() {
        shooterUpper.setVelocity(velocityTarget);
        shooterLower.setVelocity(velocityTarget);
    }

    public void setVelocityTarget(double targetInRPS) {
        velocityTarget = targetInRPS * TPR;
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

    public void waitForShoot(double velocity) { // no test
        //shootByVelocity(velocity);
        while (getVelocityRPS() >= velocityTarget + ERROR || getVelocityRPS() <= velocityTarget - ERROR)
            ;
        openCover();
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

    public double getAngleAdjusterPos() {
        return angleAdjuster.getPosition();
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                setShortThrowMode();
                setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
                shootByVelocity();
                while (timer.milliseconds() < 23000) ;
                shooterUpper.setVelocity(0);
            }
        }
    }

    public class ShooterPortion extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                if (needShootPortion) {
                    closeCover();
                    setLongThrowMode();
                    setVelocityTarget(VELOCITY_FOR_LONG_THROW);
                    shootByVelocity();
                    timer.reset();
                    sleeper(TIME_FOR_SET_VELOCITY);
                    for (int i = 0; i < 3; i += 1) {//rewrite for shooter
                        openCover();
                        timer.reset();
                        sleeper(TIME_GATES_BETWEEN_SHOOT);
                        closeCover();
                        timer.reset();
                        sleeper(TIME_GATES_BETWEEN_SHOOT);
                    }
                    needShootPortion = false;
                }
            }
        }
    }

    public void needShootPortion() {
        this.needShootPortion = true;
    }

    public void setVelocityAuto(double RPS) { //For ContinuousShooter
        velocityTarget = RPS * TPR;
    }

    public void updateCalculator(double RPS) {
        if (artifactsNow == 3) artifactsNow = 0;

        if (isShoot(RPS) && RPS != 0) {
            canShoot = false;
            artifacts++;
            artifactsNow++;
        } else {
            timers = 50;
            canShoot = true;
        }
    }

    public double getPower() {
        return shooterUpper.getPower();
    }

    public void sleeper(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }

    public boolean isShoot(double RPS) {
        return getVelocityRPS() > RPS - 4;
    }

    public boolean isBack(double RPS) {
        return getVelocityRPS() >= (RPS - 1); //погрешность подобрать
    }
}

