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
    DigitalChannel magneticSensor;
    Limelight camera;
    public TurretRegulator turretRegulator = new TurretRegulator();

    Alliance alliance = Alliance.NONE;

    public static double POWER = 1;

    public final double rSmallGear = 60;
    public final double rBigGear = 178;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private final double TPR = 537.7;
    public double error;
    public double dError;
    public double sumError = 0;
    public double pastError;
    public double target = 0;
    public double angleOfTurret;
    public static double POS_RIGHTMOST = 360;  //подобрать
    public static double POS_LEFTMOST = -360;

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
        if (isMagneting()) {
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

                double powerP = error * kP + sumError * kI + dError * kD / timer.milliseconds();

                turnInLimits(powerP);

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
        switch(alliance) {
            case RED:
                angleOfTurret = Math.atan((144-y)/(144-x));
                if (angleOfDrivetrain > 180) {
                    angleOfDrivetrain = 360 - angleOfDrivetrain;
                }
                if (angleOfDrivetrain >= angleOfTurret) {
                    target = angleOfDrivetrain - angleOfTurret;
                } else {
                    target = angleOfDrivetrain + angleOfTurret;
                }

            case BLUE:
                angleOfTurret = Math.atan((144-y)/x);
                if (angleOfDrivetrain > 180 - angleOfTurret && angleOfDrivetrain < 360 - angleOfTurret) {
                    target = angleOfDrivetrain - (180 - angleOfTurret);
                } else {
                    if (angleOfDrivetrain > 0 && angleOfDrivetrain < 180 - angleOfTurret) {
                        target = 180 - (angleOfTurret + angleOfDrivetrain);
                    } else {
                        target = (360 - angleOfDrivetrain) + (180 - angleOfTurret);
                     }
                }
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
