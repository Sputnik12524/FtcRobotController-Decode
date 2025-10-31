package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST Shooter/Sorting AND INTAKE", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Intake in;
    Sorting sr;
    boolean bState = false;
    boolean xState = false;
    boolean yState = false;

    @Override
    public void runOpMode() {
        in = new Intake(this);
        sr = new Sorting(this);
        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.hwallClose();
        sr.dwallOpen();

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                in.rotateIn(1);
                sr.autoDrumTurningIn(0.5);
            } else {
                in.rotateStop();
                sr.drumStop();
            }
            if (gamepad1.b) {
                in.rotateIn(1);
                sr.autoDrumTurningIn(0.6);
            } else {
                in.rotateStop();
                sr.drumStop();
            }
            if (gamepad1.y) {
                in.rotateIn(1);
                sr.autoDrumTurningIn(0.7);
            } else {
                in.rotateStop();
                sr.drumStop();
            }
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition());
            telemetry.addData("Позиция стенки", sr.hwall.getController());
            telemetry.update();
        }
//        sr.sortIntaker.interrupt();
//        sr.sortShooter.interrupt();
//        sr.sortMotorDriver.interrupt();
    }
}
