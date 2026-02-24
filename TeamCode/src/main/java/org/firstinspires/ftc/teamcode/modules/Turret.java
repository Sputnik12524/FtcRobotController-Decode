package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
public class Turret {
    LinearOpMode opMode;
    public DcMotorEx turret;
    Limelight limelight3A;
    DigitalChannel magneticSensor;
    Alliance alliance = Alliance.NONE;
    public TurretRegulator turretRegulator = new TurretRegulator();

    public final double rSmallGear = 60;
    public final double rBigGear = 178;

    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.01;
    private final double TPR = 537.7;
    public double error;
    public double dError;
    public double sumError = 0;
    public double pastError;
    public double target = 0;
    public double angleOfTurret;
    public static double POS_RIGHTMOST = 180;
    public static double POS_LEFTMOST = -180;

    private boolean stateMagneting = false;

    public boolean isInLimits = false;
    public TurretCameraAiming cameraAiming = new TurretCameraAiming();

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
                error = target - getCurrentPosOfTurret();
                dError = error - pastError;
                sumError = sumError + error * getCurrentPosOfTurret();

                double power = error * kP + sumError * kI + dError * kD / timer.milliseconds();

                turnInLimits(power);

                pastError = error;
                timer.reset();
            }
        }
    }

    public class TurretCameraAiming extends Thread {

        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            timer.reset();
            while (!isInterrupted()) {

                error = -limelight3A.getTagInfo().get(1);
                // error = центр - координаты_эйприл_тега

                double powerP = error * kP;

                turnInLimits(powerP);

                timer.reset();
            }
        }
    }



    public double getCurrentPosOfTurret() {
        return turret.getCurrentPosition()/TPR * rSmallGear/rBigGear * 360;
    }

    public void turnInLimits(double power) {
        if (isMagneting() && !stateMagneting) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else if (getCurrentPosOfTurret() > POS_RIGHTMOST && power > 0) {
            isInLimits = false;
            turnStopByPower();
        } else if (getCurrentPosOfTurret() < POS_LEFTMOST && power < 0) {
            isInLimits = false;
            turnStopByPower();
        } else {
            isInLimits = true;
            turret.setPower(power);
        }
        stateMagneting = isMagneting();
    }


    public boolean isMagneting() {
        return !magneticSensor.getState();
    }

    public void turnByTarget(double target) { this.target = target; }
    public double stabilizeTargetByCamera(double tar){
        return (tar - limelight3A.getGoalTag().get(1));
    }

    public void continuousTurnToGate(Alliance alliance, double x, double y, double angleOfDrivetrain) {
        if (x <= 0) x = 1;
        if (x >= 144) x = 143;
        switch(alliance) {
            case RED:
                angleOfTurret = Math.toDegrees(Math.atan((144-y)/(144-x)));
                break;
            case BLUE:
                angleOfTurret = 180 - Math.toDegrees(Math.atan((144-y)/x));
                break;
        }
        target = -(angleOfDrivetrain - angleOfTurret);
        target -= 180;
        target = angleNormalising(target);
    }
    public double angleNormalising(double targetNew) {
        double normTarget = targetNew;
        if (targetNew > 180) {
            normTarget = targetNew - 360;
        } else if (targetNew < -180) {
            normTarget = targetNew + 360;
        }
        return normTarget;
    }

    public void turnStopByPower() {
        turret.setPower(0);
    }

}
