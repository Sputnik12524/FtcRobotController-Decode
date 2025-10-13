package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Parking {
    private DcMotor upMotor;
    private Servo servo;

    public static double POWER = 1;
    public static double UP = 1000;
    public static double POS = 0;



    public Parking(LinearOpMode linearOpMode) {
        this.upMotor = linearOpMode.hardwareMap.get(DcMotor.class, "upMotor");
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//??  Не уверена сюда ли направление
        upMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setPower() {
        upMotor.setPower(POWER);
    }
}
