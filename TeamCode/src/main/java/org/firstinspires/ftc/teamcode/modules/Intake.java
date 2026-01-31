package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config

public class Intake {

    public final DcMotorEx catcher;
    public ArtifactIntake artifactIntake;
    LinearOpMode linearOpMode;
    CRServo transferServo;


    public static double POWER = 1;
    public static double IN_OUT = 300;
    public static double TRANSFER_POWER = 1;

    public Intake(LinearOpMode linearOpMode) {
        artifactIntake = new ArtifactIntake();
        this.linearOpMode = linearOpMode;
        this.catcher = linearOpMode.hardwareMap.get(DcMotorEx.class, "catcher");
        transferServo = linearOpMode.hardwareMap.get(CRServo.class,"transferServo");
    }

    public void rotateIn() {
        catcher.setPower(POWER);
        transferSetPower(1);
    }

    public void rotateOut() {
        catcher.setPower(-POWER);
        transferSetPower(-1);
    }

    public void rotateStop() {
        catcher.setPower(0);
        transferServo.setPower(0);
    }


    public void needRotateIn() {
        artifactIntake.isRotateIn = true;
    }

    public void needRotateOut() {
        artifactIntake.isRotateOut = true;
    }
    public void transferSetPower(double power){
        transferServo.setPower(power);
    }

    public class ArtifactIntake extends Thread {
        private final ElapsedTime timer = new ElapsedTime();
        boolean isRotateIn = true;
        boolean isRotateOut = false;

        public void run() {
            while (!isInterrupted()) {
                   for (int i = 0; i < 4; i++) {
                       timer.reset();
                       rotateIn();
                       while (timer.milliseconds() < IN_OUT);
                       rotateStop();
                       timer.reset();
                       while (timer.milliseconds() < 1500);
                   }
                   rotateIn();
            }
        }
    }
}
