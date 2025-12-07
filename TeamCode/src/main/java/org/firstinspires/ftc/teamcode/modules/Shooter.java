package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter { //sh

    public final DcMotorEx shooter;
    public final DcMotorEx shooterTest;
    public final double TPR = 28;

    public static double POWER = 1;
    public static double VELOCITY = 0;
    public static double VELO_HUMAN = 45;
    public static double VELO_GOAL = 37;

   public ContinuousShooter continuousShooter = new ContinuousShooter();

    public Shooter(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooterTest = opMode.hardwareMap.get(DcMotorEx.class, "shooterTest");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootByVelocity(double RPS) {
        shooter.setVelocity(RPS*TPR);
    }
    public void shootByPower(double POWER) {
        shooter.setPower(POWER);
    }
    public void setPower(double POWER) {
        shooter.setPower(POWER);
    }
    public void shootStop() {
        //shooter.setVelocity(0);
        shooter.setPower(0);
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                shooter.setVelocity(VELOCITY);
                while (timer.milliseconds() < 23000);
                shooter.setVelocity(0);
            }
        }
    }
    public void setVelocityAuto(double RPS){ //For ContinuousShooter
        VELOCITY = RPS * TPR;
    }

    public double getPower() {
        return shooter.getPower();
    }
    public double getVelocityRPS() {
        return shooter.getVelocity()/TPR;
    }
    public double getVelocityTPS() {
        return shooter.getVelocity();
    }


}
