package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {

    public final DcMotorEx shooter;
    public final DcMotorEx shooterUp;
    private final Servo cover;
    public static double TURNOVER = 360;

    public static double POWER = 1; // было 0.83
    public static double POWER2 = 1;
    public static double SHORT_THROW = 0.1;
    public static double LONG_THROW = 0.2;
    public static double target;
    public static double error;

    private boolean isItShortThrow = false;
   public ContinuousShooter continuousShooter = new ContinuousShooter();

    public Shooter(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooterUp = opMode.hardwareMap.get(DcMotorEx.class, "shooter2");

        cover = opMode.hardwareMap.get(Servo.class, "cover");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shootByPower(double POWER) {
        shooterUp.setPower(POWER);
    }

    public void shoot(){
        shooter.setPower(POWER);
        shooterUp.setPower(POWER);
    }

    public void shootStop() {
        shooter.setPower(0);
        shooterUp.setPower(0);
    }

    public void setVelocityUp (double TURNOVER) {
        shooterUp.setVelocity(TURNOVER*28);
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

    public void setPower(double power){
        POWER = power;
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                shooter.setPower(POWER);
                while (timer.milliseconds() < 23000) {}
                shooter.setPower(0);
            }
        }
    }
}
