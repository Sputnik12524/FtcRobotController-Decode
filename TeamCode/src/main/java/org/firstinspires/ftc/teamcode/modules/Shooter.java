package org.firstinspires.ftc.teamcode.modules;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Shooter {
    public final DcMotorEx shooterUpper;
    LinearOpMode opMode;
    public final DcMotorEx shooterLower;
    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    Follower follower;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(26, 0, 10, 15);
    private final double TPR = 28;
    public short artifacts = 0;
    public short artifactsIn = 0;
    public short artifactsNow = 0;
    public int timers;
    public static double POWER = 1;
    public double velocityTarget = 0;
    public static double VELOCITY_FOR_LONG_THROW = 50;
    public static double VELOCITY_FOR_SHORT_THROW = 38.5;

    //Cover
    public static double POS_COVER_OPEN = 0.4;
    public static double POS_COVER_CLOSE = 0.9;

    //Adjuster
    public static double POS_SHORT_THROW = 0.05;
    public static double POS_LONG_THROW = 0.01;
    public static double TIME_BETWEEN_SHOOT = 130;
    public static double TIME_AFTER_SHOOT = 300;
    public static double DELTA_ADJUSTER = 0.01;
    public static double DELTA_SECOND_SHOOT = 0.02;
    public static double DETECT_SHOOT = 3.5;
    public static double IS_SPIN_UP = 2.7;

    public static double HIGH_OF_ROBOT = 35;
    public static double HIGH_OF_GATE = 136;
    public static double G = 9.8;
    public boolean isShooting = false;
    public boolean detected = false;
    public boolean complete = false;
    public boolean completeC = false;
    boolean isSpinUp = false;

    private final ElapsedTime timer = new ElapsedTime();

    boolean InZone = true;
    public static boolean isTunnelOpen;
    public double shootPos = 0;

    enum states {DEFAULT, INIT, SHOOT, UPDATE, RESTART, START}

    states state = states.INIT;
    private final ElapsedTime timerSh = new ElapsedTime();
    private Pose currentPose;

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
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

    public Shooter(LinearOpMode opMode, Follower follower) {
        this.follower = follower;
        this.opMode = opMode;
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");
        //  currentPose = follower.getPose();

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

    public void shootByPower() {
        shooterUpper.setPower(POWER);
        shooterLower.setPower(POWER);
    }

    public void shootStop() {
        shooterUpper.setVelocity(0);
        shooterLower.setVelocity(0);
        isShooting = false;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void setShortThrowMode() {
        angleAdjuster.setPosition(POS_SHORT_THROW);
    }

    public void setLongThrowMode() {
        angleAdjuster.setPosition(POS_LONG_THROW);
    }

    public void openTunnel() {
        cover.setPosition(POS_COVER_OPEN);
        isShooting = true;

    }

    public void closeTunnel() {
        cover.setPosition(POS_COVER_CLOSE);
    }

    public double getAngleAdjusterPos() {
        return angleAdjuster.getPosition();
    }

    public boolean ifNotInLaunchZoneGoal() {
        currentPose = follower.getPose();
        return follower.getPose().getY() <= Math.abs(follower.getPose().getX() - 72) + 61;
    }

    public boolean ifNotInLaunchZoneHuman() {
        currentPose = follower.getPose();
        return follower.getPose().getY() >= -Math.abs(follower.getPose().getX() - 72) + 24;
    }


    public void setMode(double pos) {
        if (pos < 0) angleAdjuster.setPosition(0);
        else angleAdjuster.setPosition(pos);
        // else angleAdjuster.setPosition(Math.max(pos, ADJUSTER_LIMIT));
    }

    public void threeArtefactsShooting() {
        updateCalculator();
        if (complete) {
            shootPos = angleAdjuster.getPosition();
            timer.reset();
            while (timer.milliseconds() < TIME_BETWEEN_SHOOT) {
            }
            setMode(angleAdjuster.getPosition() + DELTA_ADJUSTER);
            timer.reset();
            while (timer.milliseconds() < TIME_BETWEEN_SHOOT) {
            }
            setMode(angleAdjuster.getPosition() + DELTA_SECOND_SHOOT);
            complete = false;
            timer.reset();
            while (timer.milliseconds() < TIME_AFTER_SHOOT) {
            }
            setMode(shootPos);
            completeC = true;
        }
    }

    public void autoStupidSetVelocityAndAngle(double y) {
        if (y < 48) {
            setLongThrowMode();
            setVelocityTarget(VELOCITY_FOR_LONG_THROW);
        } else if (y > 84) {
            setShortThrowMode();
            setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
        }
    }


    public void updateCalculator() {
        if (isDetected() && detected) {
            detected = false;
            artifacts++;
            complete = true;
        }
        if (isSpinUp() && !detected) {
            detected = true;
        }
    }

    public boolean isDetected() {
        return getVelocityRPS() < velocityTarget / TPR - DETECT_SHOOT;
    }

    public boolean isSpinUp() {
        if (getVelocityRPS() == 0) return false;
        return getVelocityRPS() >= velocityTarget / TPR - IS_SPIN_UP; //погрешность подобрать
    }

    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }

    public void sleeping(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


    public void transit(states state) {
        timer.reset();
        this.state = state;
    }


   public double setContinuousVelocityByLocalize(Alliance alliance, double angleOfAdjuster, double x, double y) {
        switch (alliance) {
            case BLUE:
                return Math.sqrt(G * Math.sqrt(x)) / (2 * Math.sin(angleOfAdjuster) * Math.sin(angleOfAdjuster) * (x - (Math.cos(angleOfAdjuster) * (144 - y))));

            case RED:
                return Math.sqrt(G * Math.sqrt(144 - x)) / (2*Math.cos(angleOfAdjuster)*Math.sin(angleOfAdjuster) * ((144 - x) - (Math.cos(angleOfAdjuster) * (144 - y))));

            default:
                return velocityTarget;
        }

    }

}
