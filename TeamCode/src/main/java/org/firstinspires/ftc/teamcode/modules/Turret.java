package org.firstinspires.ftc.teamcode.modules;


import com.acmerobotics.dashboard.FtcDashboard;
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
    DigitalChannel magneticSensor;
    Limelight camera;
    public TurretRegulator turretRegulator = new TurretRegulator();

    Alliance alliance = Alliance.NONE;

    public static double POWER = 1;

    public final double rSmallGear = 60;
    public final double rBigGear = 178;

    public static double kP = 0.035;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double kF = 0;
    private final double TPR = 537.7;
    public double error;
    public double dError;
    public double sumError = 0;
    public double pastError;
    public double target = 0;
    public double angleOfTurret;
    public static double POS_RIGHTMOST = 180;  //подобрать
    public static double POS_LEFTMOST = -180;

    public static double TURRET_ZERO = 0;
    public static double TURRET_MAX = 180;
    public static double TURRET_BLUE = 25;
    public static double TURRET_RED = -22;
    private boolean stateMagneting = false;

    public boolean isInLimits = false;

    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        magneticSensor = opMode.hardwareMap.get(DigitalChannel.class, "magneticSensor");

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
            //FtcDashboard.getInstance().getTelemetry().addLine("Максимальное право");
        } else if (getCurrentPosOfTurret() < POS_LEFTMOST && power < 0) {
            isInLimits = false;
            turnStopByPower();
            //FtcDashboard.getInstance().getTelemetry().addLine("Максимальное лево");
        } else {
            isInLimits = true;
            turret.setPower(power);
            //FtcDashboard.getInstance().getTelemetry().addLine("Еду в пределах нужного");
        }
        stateMagneting = isMagneting();
    }


    public boolean isMagneting() {
        return !magneticSensor.getState();
    }

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

                /* FtcDashboard.getInstance().getTelemetry().addData("target:", target);
                FtcDashboard.getInstance().getTelemetry().addData("error:", error);
                FtcDashboard.getInstance().getTelemetry().addData("power:", power);
                FtcDashboard.getInstance().getTelemetry().addData("CurrentPos:", getCurrentPosOfTurret());
                FtcDashboard.getInstance().getTelemetry().addData("Encoders:", turret.getCurrentPosition());
                FtcDashboard.getInstance().getTelemetry().update(); */
            }
        }
    }

    public class TurretCameraAiming extends Thread {

        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            timer.reset();
            while (!isInterrupted()) {
                while(camera.getTagInfo().get(0) != 21){
                   turnRightByPower();
                }

                error = target - getCurrentPosOfTurret();
                // error = центр - координаты_эйприл_тега

                double powerP = error * kP;

                turnInLimits(powerP);

                   timer.reset();
            }
        }
    }
    public void turnRightByTarget() {
        this.target += 1;
    }
    public void turnLeftByTarget() {
        this.target -= 1;
    }
    public void turnByTarget(double target) { this.target = target; }

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
        target = -(angleOfDrivetrain - angleOfTurret); //бабах в градусы НАДО ПЕРЕВОДИЬТЬ
        target -= 180;
        angleNormalising();
    }
    public void angleNormalising() {
        if (target > 180) {
            target -= 360;
        } else if (target < -180) {
            target += 360;
        }
    }

    public void turnLeftByPower() {
        turret.setPower(POWER);
    }
    public void turnRightByPower() {
        turret.setPower(-POWER);
    }
    public void turnStopByPower() {
        turret.setPower(0);
    }

}
