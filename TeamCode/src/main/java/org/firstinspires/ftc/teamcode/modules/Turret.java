package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Turret {
    LinearOpMode opMode;
    DcMotorEx turret;
    DigitalChannel magneticSensor;
    public TurretRegulator turretRegulator = new TurretRegulator();

    public static double POWER = 1;

    public static double kP = 0.001;
    public double error;
    public double target = 0;
    public static double POS_RIGHTMOST = 360;  //подобрать
    public static double POS_LEFTMOST = -360;
    public static double POS_ZERO = 0;


    public boolean isInLimits = false;

    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        magneticSensor = opMode.hardwareMap.get(DigitalChannel.class, "magneticSensor");



        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public double getCurrentPosOfTurret() {
        return turret.getCurrentPosition();
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
            timer.reset();
            while (!isInterrupted()) {
                error = target - getCurrentPosOfTurret();

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
    public void continuousTurnGate() {

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
