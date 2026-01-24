package org.firstinspires.ftc.teamcode.opmodes.test.drivetraintests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@TeleOp(name="TEST Wheel", group = "5")
public class WheelTest extends LinearOpMode {
    DriveTrain dt;
    double SPEED = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {

        dt = new DriveTrain(this);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                dt.leftFront.setPower(SPEED);
                telemetry.addLine("LEFT FRONT");
            } else if(gamepad1.dpad_left){
                dt.leftBack.setPower(SPEED);
                telemetry.addLine("LEFT BACK");
            }else if(gamepad1.dpad_right){
                dt.rightFront.setPower(SPEED);
                telemetry.addLine("RIGHT FRONT");
            }else if(gamepad1.dpad_down){
                dt.rightBack.setPower(SPEED);
                telemetry.addLine("RIGHT BACK");
            } else {
                dt.leftBack.setPower(0);
                dt.leftFront.setPower(0);
                dt.rightBack.setPower(0);
                dt.rightFront.setPower(0);
            }
            telemetry.update();

        }
    }
}
