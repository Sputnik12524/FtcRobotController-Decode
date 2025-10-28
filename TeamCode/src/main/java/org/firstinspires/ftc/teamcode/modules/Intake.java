package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Config

public class Intake {
    private DcMotor catcher;
    private Servo flap;

    public static double POWER;
    public static double OPEN;
    public static double CLOSE;

    public Intake(LinearOpMode linearOpMode) {
        this.catcher = linearOpMode.hardwareMap.get(DcMotor.class, "catcher");
        this.flap = linearOpMode.hardwareMap.get(Servo.class, "flap");
        catcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotationIn(double time) throws InterruptedException {
        catcher.setPower(POWER);
        wait(5);
        catcher.setPower(0);
        catcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void rotationOut() throws InterruptedException {
        catcher.setPower(-POWER);
        wait(5);
        catcher.setPower(0);
        catcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void flapOpenAndClose() {
        if (flap.getPosition() == CLOSE) {
            flap.setPosition(OPEN);
        } else {
            flap.setPosition(CLOSE);
        }
    }
}