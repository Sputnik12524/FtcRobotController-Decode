package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class DriveTrain {
    public final DcMotor leftFront;
    public final DcMotor leftBack;
    public final DcMotor rightFront;
    public final DcMotor rightBack;

    public static final double WHEEL_DIAMETER = 10.1;
    public static double MODE_SLOW_POWER = 0.5;
    public static final double PULSES = 537.7;
    public static final double CENTI_TO_PULSES = PULSES / (Math.PI * WHEEL_DIAMETER);
    public static double multiplier = 1;

    public DriveTrain(LinearOpMode opMode) {
        this.leftFront = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        this.leftBack = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        this.rightFront = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        this.rightBack = opMode.hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance);
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

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance);

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

        while (leftFront.getCurrentPosition() < CENTI_TO_PULSES * distance);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public enum RobotDirection {FORWARD, BACK, RIGHT, LEFT, FORWARD_LEFT, FORWARD_RIGHT, BACK_LEFT, BACK_RIGHT, ROTATION_CLOCKWISE, COUNTERCLOCKWISE_ROTATION}
    public void setPower (double main, double side, double rotation){
        leftFront.setPower(multiplier * (main + side + rotation));
        leftBack.setPower(multiplier * (main - side + rotation));
        rightFront.setPower(multiplier * (main - side - rotation));
        rightBack.setPower(multiplier * (main + side - rotation));
    }
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

        }

    }
    public void setMotorsPower(double main, double side, double rotation) {
        leftFront.setPower(multiplier * (main + side + rotation));
        leftBack.setPower(multiplier * (main - side + rotation));
        rightFront.setPower(multiplier * (main - side - rotation));
        rightBack.setPower(multiplier * (main + side - rotation));
    }
    public void turnRightSlowMode() {
        leftFront.setPower(MODE_SLOW_POWER);
        leftBack.setPower(MODE_SLOW_POWER);
        rightFront.setPower(-MODE_SLOW_POWER);
        rightBack.setPower(-MODE_SLOW_POWER);
    }
    public void turnLeftSlowMode() {
        leftFront.setPower(-MODE_SLOW_POWER);
        leftBack.setPower(-MODE_SLOW_POWER);
        rightFront.setPower(MODE_SLOW_POWER);
        rightBack.setPower(MODE_SLOW_POWER);
    }
}
