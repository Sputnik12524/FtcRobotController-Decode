package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private DcMotor motor;
    private Servo cover;

    public static double POWER = 1;
    public static double SHORT_THROW = 0;
    public static double LONG_THROW = 1;
    private boolean isItShortThrow = false;

    public Shooter(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotor.class, "shooter");
        cover = opMode.hardwareMap.servo.get("cover");
    }

    public void shoot() {
        motor.setPower(POWER);
    }

    public void setLongThrow() {
        cover.setPosition(LONG_THROW);
        isItShortThrow = false;
    }

    public void setShortThrow() {
        cover.setPosition(SHORT_THROW);
        isItShortThrow = true;
    }

    public void switchTrowMode() {
        if (isItShortThrow) {
            cover.setPosition(LONG_THROW);
            isItShortThrow = false;
        } else {
            cover.setPosition(SHORT_THROW);
            isItShortThrow = true;
        }
    }


}
