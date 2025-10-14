package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor Catch;
    private Servo flap;

    public static double POWER;
    public static double MAX;
    public static double OPEN;
    public static double CLOSE;

    public Intake(LinearOpMode linearOpMode) {
        this.Catch = linearOpMode.hardwareMap.get(DcMotor.class, "catch");
        this.flap = linearOpMode.hardwareMap.get(Servo.class, "flap");
        Catch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Catch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotation() {
        if (Catch.getCurrentPosition() == 0) {
            Catch.setPower(POWER);
        } else if (Catch.getCurrentPosition() >= MAX) {
            Catch.setPower(-POWER);
        }
    }
    public void flap (){
        if (flap.getPosition() >= 0) {
            flap.setPosition(OPEN);
        } else {
            flap.setPosition(CLOSE);
        }
    }
}
