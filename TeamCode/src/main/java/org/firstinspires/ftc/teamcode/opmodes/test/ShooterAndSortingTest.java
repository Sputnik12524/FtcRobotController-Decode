package org.firstinspires.ftc.teamcode.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (group = "Robot")
public class ShooterAndSortingTest extends LinearOpMode {
    DcMotor motor;
    double power = 1;

    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                motor.setPower(power);
            } else if (gamepad1.b) {
                motor.setPower(-power);
            } else {
                motor.setPower(0);
            }

        }

    }
}

