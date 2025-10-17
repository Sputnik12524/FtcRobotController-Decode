package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public static double WHEEL_DIAMETER = 10.1;
    public static double PULSES = 537.7;
    public static double CENTI_TO_PULSES = PULSES / (Math.PI * WHEEL_DIAMETER);


    public DriveTrain(LinearOpMode linearOpMode) {
        this.leftFront = linearOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        this.leftBack = linearOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        this.rightFront = linearOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        this.rightBack = linearOpMode.hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void driveToDistanceStraight(double distance, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance) {
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void driveToDistanceSides(double distance, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(power);

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void driveToDistanceRotation(double distance, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public enum RobotDirection {FORWARD, BACK, RIGHT, LEFT, FORWARD_LEFT, FORWARD_RIGHT, BACK_LEFT, BACK_RIGHT, ROTATION_CLOCKWISE, COUNTERCLOCKWISE_ROTATION}

    public void setDTPower(RobotDirection direction, double power, double distance) {
        switch (direction) {

            case FORWARD:
                driveToDistanceStraight(distance, power);
                break;

            case BACK:
                driveToDistanceStraight(distance, -power);
                break;

            case LEFT:
                driveToDistanceSides(distance, -power);
                break;

            case RIGHT:
                driveToDistanceSides(distance, power);
                break;

            case ROTATION_CLOCKWISE:
                driveToDistanceRotation(distance, power);
                break;

            case COUNTERCLOCKWISE_ROTATION:
                driveToDistanceRotation(distance, -power);
                break;
/*
            case FORWARD_LEFT:
                leftFront.setPower(0);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(0);
                break;

            case FORWARD_RIGHT:
                leftFront.setPower(power);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(power);
                break;

            case BACK_LEFT:
                leftFront.setPower(-power);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(-power);
                break;

            case BACK_RIGHT:
                leftFront.setPower(0);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(0);
                break;
 */

        }
    }
}
