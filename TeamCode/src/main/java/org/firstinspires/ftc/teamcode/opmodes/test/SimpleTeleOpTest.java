package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@TeleOp(name="Simple TeleOp")
public class SimpleTeleOpTest extends LinearOpMode {
    DriveTrain dt = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        double main = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x;
        double rotate = gamepad1.left_trigger - gamepad1.right_trigger;

        if(isStopRequested()) return;

        dt.setMotorPower(main,side,rotate);

        telemetry.addData("main", main);
        telemetry.addData("side", side);
        telemetry.addData("rotate", rotate);
        telemetry.update();
    }
}
