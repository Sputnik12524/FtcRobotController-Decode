package org.firstinspires.ftc.teamcode.modules;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
public class Turret {
    final LinearOpMode opMode;
    public final DcMotorEx turret;
    Limelight limelight3A;
    final DigitalChannel magneticSensor;
    Alliance alliance = Alliance.NONE;
    AimingMethod aimMethod = AimingMethod.NONE;
    public TurretRegulator turretRegulator = new TurretRegulator();

    public final double rSmallGear = 60;
    public final double rBigGear = 178;

    public static double kPC = 0.0189; //0.015
    public static double kDC = 0.00015;

    public static double kPL = 0.021; //0.02

    public static double kIL = 0;
    public static double kDL = 0.025;
    public static double kF = 0.1;
    private final double TPR = 537.7;
    //public double current;
    public double error;
    public double dError;
    public double sumError = 0;
    public double pastError;
    public double target = 0;
    public double ll_weight = 0;
    public double angleOfTurret;
    public static double POS_RIGHTMOST = -100;
    public static double POS_LEFTMOST = 275;

    public static double delta = 0.05;

    private boolean stateMagneting = false;
    public boolean isResetTurretPose = false;


    public boolean isInLimits = false;
    public double voltage;
    public double errorPlus;
    public double ZeroRealPose;
    double currentPoseOfTurret;

    public Turret(LinearOpMode opMode, Limelight ll) {
        this.opMode = opMode;
        limelight3A = ll;
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        magneticSensor = opMode.hardwareMap.get(DigitalChannel.class, "magneticSensor");

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        magneticSensor = opMode.hardwareMap.get(DigitalChannel.class, "magneticSensor");

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    //---------------------------------------------- BY LOCALIZATION
    public class TurretRegulator extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            timer.reset();
            double power;
            double powerP;
            while (!isInterrupted()) {
                currentPoseOfTurret = getCurrentPosOfTurret();
                switch (aimMethod) {
                    case TO_ZERO:
                        error = -currentPoseOfTurret - ZeroRealPose;
                        turnToZeroPosition(error*kPL);
                        break;
                    case CAMERA:
                        //sumError = 0; //
                        BigDecimal bd = new BigDecimal(Double.toString(-limelight3A.getTagInfo()[1]));
                        bd = bd.setScale(3, RoundingMode.HALF_UP);
                        error = bd.doubleValue();
                        if (error < 2 && error > -2) turnInLimits(0);
                        else {
                            dError = error - pastError;

                              powerP = error * kPC + kDC * dError / timer.milliseconds(); //ПРОСТО ПАВЕРП ЕСЛИ ПЛАВНОЕ

                            turnInLimits(powerP); // ЗАКОММЕНТИТЬ ЕСЛИ ПЛАВНОЕ ПЕРЕКЛЮЧЕНИЕ
                        }
                        //pastError = error; //
                        timer.reset();
                        break;

                    case LOCALIZATION:
                        error = target - currentPoseOfTurret;
                        dError = error - pastError;
                        sumError = sumError + error * currentPoseOfTurret;

                        power = error * kPL + sumError * kIL + dError * kDL / timer.milliseconds(); //JUST POWER

                        turnInLimits(power); //COMMENT IF BLEND

                        pastError = error;
                        timer.reset();

                        break;

                    case NONE:
                        turnInLimits(0);
                        timer.reset();

                }

            }
        }
    }


    public double getCurrentPosOfTurret() {
        return turret.getCurrentPosition() / TPR * rSmallGear / rBigGear * 360;
    }

    public void turnInLimits(double power) {
        double pose = getCurrentPosOfTurret();
        if (isMagneting() && !stateMagneting) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            kPL = 0.02;
        } else if (pose < POS_RIGHTMOST && power < 0) {
            isInLimits = false;
            turnStopByPower();
        } else if (pose > POS_LEFTMOST && power > 0) {
            isInLimits = false;
            turnStopByPower();
        } else if (pose > -15 && pose < 15) { //!!!!!!! ПОМЕНЯНО ИЛИ НА И. ЗАПИШИТЕ В КОНСТАНТЫ ЫЫЫЫЫЫ
            kPL = 0.0075;
            isInLimits = true;
            turret.setPower(power);
        } else {
            isInLimits = true;
            kPL = 0.02;
            turret.setPower(power);
        }
        stateMagneting = isMagneting();
    }



    public void turnToZeroPosition(double power){

        double pose = turret.getCurrentPosition();
        if (isMagneting() && !stateMagneting) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            isResetTurretPose = true;
            kPL = 0.02;
            setAimMethod(AimingMethod.NONE);
        } else if (abs(-pose - ZeroRealPose) < 15) {
            kPL = 0.0075;
            turret.setPower(power);
        } else {
            kPL = 0.01;
            turret.setPower(power);
        }
        stateMagneting = isMagneting();

    }


    public boolean isMagneting() {
        return !magneticSensor.getState();
    }

    public void turnByTarget(double target) {
        this.target = target;
    }

    public double angleNormalising(double targetNew) {
        double normTarget = targetNew;
        if (targetNew < POS_RIGHTMOST) {
            normTarget = targetNew + 360;
        } else if (targetNew > POS_LEFTMOST) {
            normTarget = targetNew - 360;
        }
        return normTarget;
    }

    public void turnStopByPower() {
        turret.setPower(0);
    }

    public void tuneTurretPID(double kPL, double kIL, double kDL, double kPC, double kDC) {
        Turret.kPL = kPL;
        Turret.kIL = kIL;
        Turret.kDL = kDL;

        Turret.kPC = kPC;
        Turret.kDC = kDC;
    }

    public double getAmps() {
        return turret.getCurrent(CurrentUnit.AMPS);
    }

    public void setAimMethod(AimingMethod aimingMethod) {
        aimMethod = aimingMethod;
    }

    public void update(double y) {
        if (y < 50) {
            errorPlus = 3.5;
        } else errorPlus = 0;
    }

    public AimingMethod getAimMethod() {
        return aimMethod;
    }

    public double clampValue(double value, double min, double max) {
        if (value > max) value = max;
        else if (value < min) value = min;
        return value;
    }

    public double[] getLocalizationCoefficients() {
        return new double[]{
                kPL, kIL, kDL
        };
    }

    public double[] getCameraCoefficients() {
        return new double[]{
                kPC, kDC
        };
    }

}
