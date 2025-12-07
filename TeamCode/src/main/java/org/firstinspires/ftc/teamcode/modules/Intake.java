package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config

public class Intake {

    private final DcMotor catcher;

    public static double POWER = 1;

    public static double IN_OUT = 2000;


    ArtifactIntake artifactIntake;
    LinearOpMode linearOpMode;

    public Intake(LinearOpMode linearOpMode) {
        artifactIntake = new ArtifactIntake();
        this.linearOpMode = linearOpMode;
        this.catcher = linearOpMode.hardwareMap.get(DcMotor.class, "catcher");
    }

    public void rotateIn() {
        catcher.setPower(POWER);
    }

    public void rotateOut() {
        catcher.setPower(-POWER);
    }

    public void rotateStop() {
        catcher.setPower(0);
    }


    public void needRotateIn() {
        artifactIntake.isRotateIn = true;
    }

    public void needRotateOut() {
        artifactIntake.isRotateOut = true;
    }

    public class ArtifactIntake extends Thread {
        private final ElapsedTime timer = new ElapsedTime();
        boolean isRotateIn = false;
        boolean isRotateOut = false;

        public void run() {
            while (!isInterrupted()) {
                if (isRotateIn){
                    timer.reset();
                    rotateIn();
                    while (timer.milliseconds() < IN_OUT);
                    rotateStop();
                    isRotateIn = false;
                }
                if (isRotateOut){
                    timer.reset();
                    rotateOut();
                    while (timer.milliseconds() < IN_OUT);
                    rotateStop();
                    isRotateOut = false;
                }
            }
        }
    }
}
