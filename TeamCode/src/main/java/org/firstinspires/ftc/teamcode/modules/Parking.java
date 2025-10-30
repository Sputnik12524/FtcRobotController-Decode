package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config

public class Parking {
    public DcMotor upMotor;
    private Servo servo;

    public static double POWER = 1;
    public static double ENCODER_PULSES = 537.7;
    public static double WHEEL_DIAMETER = 9.6;
    public static double PULSES_CM = ENCODER_PULSES / (Math.PI * WHEEL_DIAMETER);
    public static double K = 1;
    public static double error;
    public static double target;


    public static int MAX_POSITION = 1000;
    public static int MIN_POSITION = 0;


    public Parking(LinearOpMode linearOpMode) {
        this.upMotor = linearOpMode.hardwareMap.get(DcMotor.class, "upMotor");
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Не уверена
    public void getUp() {
        upMotor.setPower(POWER);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public class ArtefactParking extends Thread {

        @Override
        public void run() {
            upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (!isInterrupted()) {
                error = target - upMotor.getCurrentPosition();
                upMotor.setPower(error * K);
            }
        }
    }
}

