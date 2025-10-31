package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {

    private final DcMotor shooter;
    private final Servo cover;

    public static double POWER = 1;
    public static double SHORT_THROW = 0.1;
    public static double LONG_THROW = 0.2;
    private boolean isItShortThrow = false;

    public Shooter(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotor.class, "shooter");
        cover = opMode.hardwareMap.get(Servo.class, "cover");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot() {
        shooter.setPower(POWER);
    }

    public void shootStop() {
        shooter.setPower(0);
    }

    public void setLongThrow() {
        cover.setPosition(LONG_THROW);
        isItShortThrow = false;
    }

    public void setShortThrow() {
        cover.setPosition(SHORT_THROW);
        isItShortThrow = true;
    }

    public void switchThrowMode() {
        if (isItShortThrow) {
            cover.setPosition(LONG_THROW);
            isItShortThrow = false;
        } else {
            cover.setPosition(SHORT_THROW);
            isItShortThrow = true;
        }
    }
}
