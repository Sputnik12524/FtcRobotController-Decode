package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Turret {
    LinearOpMode opMode;
    public DcMotorEx turret;
    Limelight limelight3A;
    DigitalChannel magneticSensor;
    Alliance alliance = Alliance.NONE;
    AimingMethod aimMethod = AimingMethod.NONE;
    public TurretRegulator turretRegulator = new TurretRegulator();

    public final double rSmallGear = 60;
    public final double rBigGear = 178;

    public static double kPC = 0.015;
    public static double kDC = 0;

    public static double kPL = 0.02;

    public static double kIL = 0;
    public static double kDL = 0.02;
    public static double kF = 0.1;
    private final double TPR = 537.7;
    //public double current;
    public double error;
    public double dError;
    public double sumError = 0;
    public double pastError;
    public double target = 0;
    public double angleOfTurret;
    public static double POS_RIGHTMOST = 225;
    public static double POS_LEFTMOST = -130;

    private boolean stateMagneting = false;

    public boolean isInLimits = false;
    public double voltage;
    public double errorPlus;

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
            while (!isInterrupted()) {
                switch (aimMethod) {
                    case CAMERA:
                        error = -limelight3A.getTagInfo().get(1);
                        if (error < 0.25 && error > -0.25) turnInLimits(0);
                        else {
                            dError = error - pastError;

                            double powerP = error * kPC + kDC * dError / timer.milliseconds();

                            turnInLimits(powerP);
                        }
//ошибка
                        timer.reset();


                        break;

                    case LOCALIZATION:
                        error = target - getCurrentPosOfTurret();
                        dError = error - pastError;
                        sumError = sumError + error * getCurrentPosOfTurret();

                        double power = error * kPL + sumError * kIL + dError * kDL / timer.milliseconds();

                        turnInLimits(power);

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
        } else if (pose > POS_RIGHTMOST && power > 0) {
            isInLimits = false;
            turnStopByPower();
        } else if (pose < POS_LEFTMOST && power < 0) {
            isInLimits = false;
            turnStopByPower();
        } else if (pose > -15 || pose < 15) {
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


    public boolean isMagneting() {
        return !magneticSensor.getState();
    }

    public void turnByTarget(double target) {
        this.target = target;
    }

    public double angleNormalising(double targetNew) {
        double normTarget = targetNew;
        if (targetNew > POS_RIGHTMOST) {
            normTarget = targetNew - 360;
        } else if (targetNew < POS_LEFTMOST) {
            normTarget = targetNew + 360;
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


}
