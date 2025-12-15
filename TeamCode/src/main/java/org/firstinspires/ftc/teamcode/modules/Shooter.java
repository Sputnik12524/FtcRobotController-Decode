package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {//sh
    private final Object monitor = new Object();
    public final DcMotorEx shooter;
    private Servo angleCover;
    private Servo shutCover;
    private final VoltageSensor batteryVoltageSensor;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTER_OLD = new PIDFCoefficients(30, 0, 30, 27);
    public static PIDFCoefficients MOTOR_VELO_PID_TURRET = new PIDFCoefficients(0, 0, 0, 0);
    private final double TPR = 28;
    public String error;
    private boolean shooting = true;
    public int artifacts = 0;
    public static final double HUMAN_SPEED = 1000;
    public static final double NOT_HUMAN_SPEED = 400;
    public static final double ERROR_WHEN_SHOOT = 28.585;
    public static final long SLEEP_AFTER_SHOOT = 500;
    public static final double SLEEP_BEFORE_MONITOR = 1500;
    public static double POWER = 1;
    public static double VELOCITY = 0;
    public static double VELO_HUMAN = 45;
    public static double VELO_GOAL = 37;
    public static double ERROR = 3;
    public static double COVER_GOAL;
    public static double COVER_HUMAN;
    public static double COVER_OPEN;
    public static double COVER_CLOSE;
    boolean StateA = false;
    boolean isShooting = false;

    public ContinuousShooter continuousShooter = new ContinuousShooter();
    public ArtefactsCalculator artefactsCalculator = new ArtefactsCalculator();

    public Shooter(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shutCover = opMode.hardwareMap.get(Servo.class, "shutCover");
        angleCover = opMode.hardwareMap.get(Servo.class, "angleCover");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(shooter, MOTOR_VELO_PID_SHOOTER_OLD);
    }

    public void shootByVelocity(double RPS) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        shooter.setVelocity(RPS * TPR);
        while (timer.milliseconds() < SLEEP_BEFORE_MONITOR) ; //подобрать
        synchronized (monitor) {
            shooting = true;
            monitor.notify();
        }
    }

    public void shootByPower(double POWER) {
        shooter.setPower(POWER);
    }

    public void shootStop() {
        shooter.setVelocity(0);
        shooting = false;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void waitForShoot(double velocity) {
        shootByVelocity(velocity);
        while (getVelocityRPS() >= VELO_GOAL + ERROR || getVelocityRPS() <= VELO_GOAL - ERROR) {
        }
        //тут открытие крышки
    }

    public void setAngleCoverGoal() {
        angleCover.setPosition(COVER_GOAL);
    }

    public void setAngleCoverHuman() {
        angleCover.setPosition(COVER_HUMAN);
    }

    public void shutCoverOpen() {
        shutCover.setPosition(COVER_OPEN);
    }

    public void shutCoverClose() {
        shutCover.setPosition(COVER_CLOSE);
    }

    public void setAngleCoverPos(double angle) {
        angleCover.setPosition(angle);
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                shooter.setVelocity(VELOCITY);
                while (timer.milliseconds() < 23000) ;
                shooter.setVelocity(0);
            }
        }
    }

    public class ArtefactsCalculator extends Thread {
        ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            while (!isInterrupted()) {
                synchronized (monitor) {
                    while (!shooting) {
                        try {
                            monitor.wait();
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
                monitoring();
            }
        }


        public void monitoring() {
            while (shooting) {
                if (isShoot()) {
                    artifacts++;
                    try {
                        Thread.sleep(SLEEP_AFTER_SHOOT);//подобрать
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }

        public boolean isShoot() {
            return shooter.getVelocity() < VELOCITY - ERROR_WHEN_SHOOT;//подобрать
        }
    }

    public void setVelocityAuto(double RPS) { //For ContinuousShooter
        VELOCITY = RPS * TPR;
    }

    public double getPower() {
        return shooter.getPower();
    }

    public double getVelocityRPS() {
        return shooter.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooter.getVelocity();
    }

    public void setHumanSpeed() {
        VELOCITY = HUMAN_SPEED;
    }

    public void setNotHumanSpeed() {
        VELOCITY = NOT_HUMAN_SPEED;
    }


}
