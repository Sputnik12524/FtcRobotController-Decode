package org.firstinspires.ftc.teamcode.opmodes.test.drivetraintests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Cycle;

@TeleOp(name="Simple TeleOp", group="3")

public class SimpleTeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain dt = new DriveTrain(this);
        Cycle cc = new Cycle();

        waitForStart();
        while (opModeIsActive()) {
            cc.update();

            double main = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            double side_input = gamepad1.left_stick_x;
            double side = Math.signum(side_input) * Math.pow(side_input, 2);
            double rotate_input = gamepad1.right_trigger - gamepad1.left_trigger;
            double rotate = Math.signum(rotate_input) * Math.pow(rotate_input, 2);



            dt.setPower(main, side, rotate);

            telemetry.addData("Max cycle",cc.getMax());
            telemetry.addData("Average cycle", cc.getAverage());
            telemetry.addData("Cycles", cc.getCycles());
            telemetry.addData("DriveTrain", dt.getAverageAmps());
            telemetry.addData("LeftBack", dt.getLbAMPS());
            telemetry.addData("LeftFront", dt.getLfAMPS());
            telemetry.addData("RightBack", dt.getRbAMPS());
            telemetry.addData("RightFront", dt.getRfAMPS());

            telemetry.addData("main", main);
            telemetry.addData("side", side);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }
}
