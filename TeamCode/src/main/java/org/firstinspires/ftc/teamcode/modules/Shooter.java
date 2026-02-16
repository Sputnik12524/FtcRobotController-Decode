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
    public final DcMotorEx shooterLower;
    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    LinearOpMode opMode;
    Follower follower;
    Pose currentPose;
    Transfer tr;

    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(26, 0, 10, 15);
    private final ElapsedTime timer = new ElapsedTime();

    enum states {DEFAULT, INIT, SHOOT, UPDATE, RESTART, START}

    states state = states.INIT;

    //---------------------------------------------- DASHBOARD

    /// Shooter
    public static double VELOCITY_FOR_LONG_THROW = 47;
    public static double VELOCITY_FOR_SHORT_THROW = 38.5;
    public static double POWER = 1;

    ///  Cover
    public static double POS_COVER_OPEN = 0.4;
    public static double POS_COVER_CLOSE = 0.9;
    public boolean canShoot = false;

    /// Adjuster
    public static double POS_SHORT_THROW = 0.05;
    public static double POS_LONG_THROW = 0.01;
    public static double TIME_BETWEEN_SHOOT = 130;
    public static double TIME_AFTER_SHOOT = 300;
    public static double DELTA_ADJUSTER = 0.01;
    public static double DELTA_SECOND_SHOOT = 0.02;
    public static double DETECT_SHOOT = 3.5;
    public static double IS_SPIN_UP = 2.7;
    public final double MAX_POS = 1;


    //---------------------------------------------- CONSTANTS
    private final double TPR = 28;
    private final double g = 9.81;

    //----------------------------------------------

    public short artifacts = 0;
    public double velocityTarget = 0;
    public double shootPos = 0;

    //---------------------------------------------- BOOLEANS
    public boolean detected = false;
    public boolean complete = false;
    public static boolean isTunnelOpen;

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        shooterUpper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterUpper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterLower.setDirection(DcMotorSimple.Direction.REVERSE);

        setPIDFCoefficients(shooterUpper, MOTOR_VELO_PID_SHOOTERS);
        setPIDFCoefficients(shooterLower, MOTOR_VELO_PID_SHOOTERS);
    }

    public Shooter(LinearOpMode opMode, Follower follower, Transfer transfer) {
        this.opMode = opMode;
        tr = transfer;
        this.follower = follower;
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        shooterUpper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterUpper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterLower.setDirection(DcMotorSimple.Direction.REVERSE);

        setPIDFCoefficients(shooterUpper, MOTOR_VELO_PID_SHOOTERS);
        setPIDFCoefficients(shooterLower, MOTOR_VELO_PID_SHOOTERS);
    }




    //---------------------------------------------- VELOCITY
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void setVelocityTarget(double targetInRPS) {
        velocityTarget = targetInRPS * TPR;
    }

    public double setContinuousVelocityByLocalize(Alliance alliance, double angleOfAdjuster, double x, double y) {
        switch (alliance) {
            case BLUE:
                return Math.sqrt(g * Math.sqrt(x)) / (2 * Math.sin(angleOfAdjuster) * Math.sin(angleOfAdjuster) * (x - (Math.cos(angleOfAdjuster) * (144 - y))));
            case RED:
                return Math.sqrt(g * Math.sqrt(144 - x)) / (2 * Math.cos(angleOfAdjuster) * Math.sin(angleOfAdjuster) * ((144 - x) - (Math.cos(angleOfAdjuster) * (144 - y))));
            default:
                return velocityTarget;
        }
    }

    public void shootByVelocity() {
        shooterUpper.setVelocity(velocityTarget);
        shooterLower.setVelocity(velocityTarget);
    }

    public void shootStop() {
        shooterUpper.setVelocity(0);
        shooterLower.setVelocity(0);
    }

    //---------------------------------------------- ADJUSTER

    public void setShortThrowMode() {
        angleAdjuster.setPosition(POS_SHORT_THROW);
    }

    public void setLongThrowMode() {
        angleAdjuster.setPosition(POS_LONG_THROW);
    }

    //---------------------------------------------- TUNNEL
    public void openTunnel() {
        isTunnelOpen = true;
        cover.setPosition(POS_COVER_OPEN);
    }

    public void closeTunnel() {
        isTunnelOpen = false;
        cover.setPosition(POS_COVER_CLOSE);
    }

    public boolean ifNotInLaunchZoneGoal() {
        currentPose = follower.getPose();
        return follower.getPose().getY() <= Math.abs(follower.getPose().getX() - 72) + 61;
    }

    public boolean ifNotInLaunchZoneHuman() {
        currentPose = follower.getPose();
        return follower.getPose().getY() >= -Math.abs(follower.getPose().getX() - 72) + 24;
    }

    public boolean inZone() {
        return ifNotInLaunchZoneHuman() || ifNotInLaunchZoneGoal();
    }

    //---------------------------------------------- CALCULATOR

    public void setMode(double pos) {
        if (pos < 0) angleAdjuster.setPosition(0);
        else if (pos > MAX_POS) angleAdjuster.setPosition(0.25);
        else angleAdjuster.setPosition(pos);
    }

    public void threeArtefactsShooting() {
        switchCover();
        if (isTunnelOpen) {
            shootPos = angleAdjuster.getPosition();
            timer.reset();
            while (timer.milliseconds() < TIME_BETWEEN_SHOOT) ;
            setMode(angleAdjuster.getPosition() + DELTA_ADJUSTER);
            timer.reset();
            while (timer.milliseconds() < TIME_BETWEEN_SHOOT) ;
            setMode(angleAdjuster.getPosition() + DELTA_SECOND_SHOOT);
            timer.reset();
            while (timer.milliseconds() < TIME_AFTER_SHOOT) ;
            setMode(shootPos);
            complete = true;
            if (tr.isEmpty()) canShoot = false;
        }
    }
    public void switchCover() {
        if (!inZone()) {
            canShoot = false;
        }
        if (canShoot) openTunnel();
        else closeTunnel();
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
        return getVelocityRPS() >= velocityTarget / TPR - IS_SPIN_UP;
    }

    //---------------------------------------------- AUTONOMOUS

    public void waitForShoot() {
        for (int i = 0; i < 5; i++) {
            opMode.sleep(2000);
            openTunnel();
            opMode.sleep(200);
            closeTunnel();
        }

    }

    //---------------------------------------------- GETTING

    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }

    public double getAngleAdjusterPos() {
        return angleAdjuster.getPosition();
    }
}
