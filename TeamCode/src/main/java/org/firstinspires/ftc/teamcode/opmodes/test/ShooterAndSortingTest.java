package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "SortingTest", group = "Robot")

public class ShooterAndSortingTest extends LinearOpMode {
    /**
     * Относиться к {@link Sorting}
     */
    Sorting sr;

    @Override
    public void runOpMode() {
        sr = new Sorting(this);
        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.wallStarting();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {  //переключение стенки
                sr.switchingWall();
            }

            sr.drumTele(gamepad1.left_stick_y / 10);
            sr.shootTele(gamepad1.right_stick_y);

            if (gamepad1.b) {
                sr.sortIntaker.start();
                while (!sr.isIntakeCompleted()) {
                }
                sr.sortIntaker.interrupt();
            }

            if (gamepad1.x) {
                sr.sortMotorDriver.start();
                while (!sr.isSortingCompleted(sr.artefact_pos(sr.getColor()), sr.pos)) { // под вопросом будет ли работать
                }
                sr.sortMotorDriver.interrupt();
            }

            if (gamepad1.y) {
                sr.sortShooter.start();
                while (!sr.isShooterCompleted()) {
                }
                sr.sortShooter.interrupt();


                if (gamepad1.dpad_right) {
                    sr.drumTele(1);
                }
            }
        }
    }
}
