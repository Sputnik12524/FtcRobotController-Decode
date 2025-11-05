package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "ServoTest", group = "Robot")
public class ServoTest extends LinearOpMode {
    public double pos = 0;
    public boolean aState = false;
    //0.65, 0 - открытие серва (верхняя)
    // 0.2, 0 - открытое, закрытое положение (нижняя)

    @Override
    public void runOpMode() {

        Sorting sorting = new Sorting(this);

        sorting.setDWallPos(0);
        telemetry.addLine("Start");


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Стенка", sorting.dwall.getPosition());
            telemetry.update();

            if (gamepad1.a && !aState) {
                pos += 0.05;
                sorting.setDWallPos(pos);
            }
            if (gamepad1.b) {
                pos -= 0.001;
                sorting.setDWallPos(pos);
            }
            if (gamepad1.x) {
                sorting.setDWallPos(0.65);
            }


            aState = gamepad1.a;


        }
    }
}




