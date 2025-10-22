package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "SortingTest", group = "Robot")

public class ShooterAndSortingTest extends LinearOpMode {
    Sorting sr;

    @Override
    public void runOpMode() {
        sr = new Sorting(this);

        sr.wallStart();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a){
                sr.switchWall();
            }
            sr.drumTele(gamepad1.left_stick_y);
            if(gamepad1.b){

            }

        }
    }
}
