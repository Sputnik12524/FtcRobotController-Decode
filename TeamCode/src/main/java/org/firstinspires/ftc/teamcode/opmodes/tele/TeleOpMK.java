package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MK", group = "A")
@Disabled
public class TeleOpMK extends LinearOpMode {

    DcMotor m0;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;
    public static double multiplier = 1;

    @Override
    public void runOpMode() {
        m0 = hardwareMap.get(DcMotor.class, "0");
        m1 = hardwareMap.get(DcMotor.class, "1");
        m2 = hardwareMap.get(DcMotor.class, "2");
        m3 = hardwareMap.get(DcMotor.class, "3");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double main = -gamepad1.left_stick_y;
            double side = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;


            m1.setPower(multiplier * (main + side + rotate)); //left front
            m2.setPower(multiplier * (main - side + rotate)); //left back
            m3.setPower(multiplier * (main - side - rotate)); //right front
            m1.setPower(multiplier * (main + side - rotate)); //right back

        }

    }
}
