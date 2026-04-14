package org.firstinspires.ftc.teamcode.opmodes.test.drivetraintests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@TeleOp(name="Simple TeleOp", group="3")

public class SimpleTeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain dt = new DriveTrain(this);

        waitForStart();
        while (opModeIsActive()) {

            double main = -gamepad1.left_stick_y;
            double side = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;


            dt.setPower(main, side, rotate);

            telemetry.addData("main", main);
            telemetry.addData("side", side);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }
}
