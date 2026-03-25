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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Shooter {
    public enum ShStates {STOP,SPINNING, SPEED_CHECK, SHOOT}
    public enum MODE{MANUAL, AUTO}
    public MODE mode = MODE.AUTO;

    ShStates state = ShStates.STOP;
    Transfer tr;
    public final DcMotorEx shooterUpper;
    public final DcMotorEx shooterLower;
    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    LinearOpMode opMode;
    Follower follower;
    Pose currentPose;

    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(12, 0, 0, 17);
    private final ElapsedTime timer = new ElapsedTime();

    enum states {DEFAULT, INIT, SHOOT, UPDATE, RESTART, START}

    public int st = 0;

    //---------------------------------------------- DASHBOARD

    /// Shooter
    public static double VELOCITY_FOR_LONG_THROW = 71;  //47 //64
    public static double VELOCITY_FOR_SHORT_THROW = 51.5;
    public static double POWER = 1;

    ///  Cover
    public static double POS_COVER_OPEN = 0.5;
    public static double POS_COVER_CLOSE = 0.8;
    public boolean canShoot = false;

    /// Adjuster
    public static double POS_SHORT_THROW = 0.05;
    public static double POS_LONG_THROW = 0.005;
    public static double TIME_BETWEEN_SHOOT = 130;
    public static double TIME_AFTER_SHOOT = 400;
    public static double DELTA_ADJUSTER = 0.01;
    public static double DELTA_SECOND_SHOOT = 0.02;
    public static double DETECT_SHOOT = 5;
    public static double IS_SPIN_UP = 7;
    public final double MAX_POS = 1;


    //---------------------------------------------- CONSTANTS
    private final double TPR = 28;
    private final double g = 9.81;

    //----------------------------------------------

    public double velocityTarget = 0;
    public double bonusLongVelocity = 0;
    public double bonusShortVelocity = 0;
    public double shootPos = 0;
    public double voltageUP;
    public double voltageLOW;
    public boolean isCanShoot = false;
    public  boolean isShootStop = false;

    //---------------------------------------------- BOOLEANS
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

    public Shooter(LinearOpMode opMode, Transfer transit) {
        this.opMode = opMode;
        tr = transit;
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

    public Shooter(LinearOpMode opMode, Follower follower, Transfer transit) {
        this.opMode = opMode;
        tr = transit;
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

    public void update() {
        if(mode == MODE.MANUAL) return;
        switch (state) {
            case STOP:
                if(!isShootStop) transit(ShStates.SPINNING);
                shootStop();
                break;

            case SPINNING:
                if (isCanShoot && inZone()) transit(ShStates.SHOOT);
                break;

            case SHOOT:
                openTunnel();
                if(timer.milliseconds() > 400){
                    closeTunnel();
                    transit(ShStates.SPINNING);
                }
        }
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

    public void shootByVelocityUpper() {
        shooterUpper.setVelocity(velocityTarget);
    }

    public void shootByVelocityLower() {
        shooterLower.setVelocity(velocityTarget);
    }

    public void shootStopUpper() {
        shooterUpper.setVelocity(0);
    }

    public void shootStopLower() {
        shooterLower.setVelocity(0);
    }

    public void shootByVelocity() {
        shooterUpper.setVelocity(velocityTarget);
        shooterLower.setVelocity(velocityTarget);
    }

    public void shootStop() {
        shooterUpper.setVelocity(0);
        shooterLower.setVelocity(0);
        closeTunnel();
    }

    //---------------------------------------------- ADJUSTER

    public void setShortThrowMode() {
        setVelocityTarget(VELOCITY_FOR_SHORT_THROW + bonusShortVelocity);
        angleAdjuster.setPosition(POS_SHORT_THROW);
        shootByVelocity();
    }

    public void setLongThrowMode() {
        setVelocityTarget(VELOCITY_FOR_LONG_THROW + bonusLongVelocity);
        angleAdjuster.setPosition(POS_LONG_THROW);
        shootByVelocity();
    }

    public void setAngleAdjuster(double angle) {
        angleAdjuster.setPosition(angle);
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
        return follower.getPose().getY() <= Math.abs(follower.getPose().getX() - 72) + 58;
    }

    public boolean ifNotInLaunchZoneHuman() {
        currentPose = follower.getPose();
        return follower.getPose().getY() >= -Math.abs(follower.getPose().getX() - 72) + 30;
    }

    public boolean inZone() {
        return !ifNotInLaunchZoneHuman() || !ifNotInLaunchZoneGoal();
    }

    //---------------------------------------------- CALCULATOR

    public void setMode(double pos) {
        if (pos < 0) angleAdjuster.setPosition(0);
        else if (pos > MAX_POS) angleAdjuster.setPosition(0.25);
        else angleAdjuster.setPosition(pos);
    }

    public void threeArtefactsShooting() {
        switch (st) {
            case 0:
                switchCover();
                shootPos = angleAdjuster.getPosition();
                if (isTunnelOpen) tr(1);
            case 1:
                setMode(angleAdjuster.getPosition() - DELTA_ADJUSTER);
                if (timer.milliseconds() > TIME_BETWEEN_SHOOT) tr(2);
            case 2:
                setMode(angleAdjuster.getPosition() - DELTA_ADJUSTER);
                if (timer.milliseconds() > TIME_BETWEEN_SHOOT) tr(3);
            case 3:
                if (timer.milliseconds() > TIME_AFTER_SHOOT) {
                    complete = true;
                    canShoot = false;
                    setMode(shootPos);
                    tr(0);
                }
        }
    }


    public void switchCover() {
        if (!inZone()) {
            canShoot = false;
        }
        if (canShoot) openTunnel();
        else closeTunnel();
    }

    public void tr(int state) {
        timer.reset();
        this.st = state;
    }

    public void setManual(boolean manual){
        mode = manual ? MODE.MANUAL: MODE.AUTO;
    }

    public void transit(ShStates state) {
        timer.reset();
        this.state = state;
    }

    public double getUpVoltage(){
        return shooterUpper.getCurrent(CurrentUnit.AMPS);
    }
    public double getLowVoltage(){
        return shooterLower.getCurrent(CurrentUnit.AMPS);
    }


    //---------------------------------------------- AUTONOMOUS
    public boolean isSpinUp() {
        if (getVelocityRPS() == 0) return false;
        return getVelocityRPS() >= velocityTarget / TPR - IS_SPIN_UP;
    }


    //---------------------------------------------- GETTING

    public double getVelocityRPSLower() {
        return shooterLower.getVelocity() / TPR;
    }

    public double getVelocityTPSLower() {
        return shooterLower.getVelocity();
    }


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
