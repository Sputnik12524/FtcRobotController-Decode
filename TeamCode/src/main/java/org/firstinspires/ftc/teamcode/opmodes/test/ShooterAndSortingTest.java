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
        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.wallStart();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {  //переключение стенки
                sr.switchWall();
            }

            sr.drumTele(gamepad1.left_stick_y);

            if (gamepad1.b) { // захват эндгейм
                while (!sr.isIntakeCompleted()) {
                    sr.sortIntaker.start();
                }
                sr.sortIntaker.interrupt();
            }

            if (gamepad1.x) { // сортировка энгейм
                while (!sr.isSortingCompleted(sr.artefact_pos(sr.getColor()), sr.pos)) { // под вопросом будет ли работать
                    sr.sortMotorDriver.start();
                }
                sr.sortMotorDriver.interrupt();
            }

            if (gamepad1.y) { // запуск эндгейм
                while (!sr.isShooterCompleted()) {
                    sr.sortShooter.start();
                }
                sr.sortShooter.interrupt();
            }
        }
    }
}
