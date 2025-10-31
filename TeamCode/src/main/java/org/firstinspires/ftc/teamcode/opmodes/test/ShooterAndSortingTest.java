package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST Shooter/Sorting", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Sorting sr;
    boolean bState = false;
    boolean xState = false;
    boolean yState = false;

    @Override
    public void runOpMode() {
        sr = new Sorting(this);
        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.hwallClose();
        sr.dwallOpen();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition());
            telemetry.addData("Позиция стенки", sr.hwall.getController());

            if (gamepad1.a) {
                sr.autoDrumTurning(0.5);
            } else if (gamepad1.b) {
                sr.autoDrumTurning(0.6);
            } else if (gamepad1.y) {
                sr.autoDrumTurning(0.7);
            }
        }
//        sr.sortIntaker.interrupt();
//        sr.sortShooter.interrupt();
//        sr.sortMotorDriver.interrupt();
    }
}
