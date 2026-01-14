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

    public static double POWER = 1;

    public static double kP = 0.001;
    public double error;
    public double target = 0;
    public static double RIGHTMOST_POS = 1234;  //подобрать
    public static double LEFTMOST_POS= 0;


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

    }

    public boolean isMagneting() {
        return magneticSensor.getState();
    }



    public class TurretRegulator extends Thread {

        @Override
        public void run() {
            while (!isInterrupted()) {
                error = target - getCurrentPosOfTurret();
                double powerP = error * kP;

                turnInLimits(powerP);
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
