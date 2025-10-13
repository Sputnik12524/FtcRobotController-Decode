package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Parking {
    private DcMotor upMotor;
    private Servo servo;

    public static double POWER = 1;
    public static double ENCODER_PULSES = 537.7;
    public static double WHEEL_DIAMETER = 9.6;
    public static double PULSES_CM = ENCODER_PULSES / (Math.PI * WHEEL_DIAMETER);


    public static int MAX_POSITION = 1000;
    public static int MIN_POSITION = 0;


    public Parking(LinearOpMode linearOpMode) {
        this.upMotor = linearOpMode.hardwareMap.get(DcMotor.class, "upMotor");
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//??  Не уверена сюда ли направление
        upMotor.setDirection(DcMotor.Direction.FORWARD);
    }
// Не уверена
    public void getUpOrDown() {
        if (upMotor.getCurrentPosition() == MAX_POSITION) {
            upMotor.setTargetPosition(MIN_POSITION);
        } else if (upMotor.getCurrentPosition() == MIN_POSITION) {
            upMotor.setTargetPosition(MAX_POSITION);
        }
    }
}
