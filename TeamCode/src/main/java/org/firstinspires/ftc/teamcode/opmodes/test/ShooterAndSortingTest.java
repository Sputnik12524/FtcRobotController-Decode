package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter shooter;
    Intake in;
    Sorting sr;
    boolean bState = false;
    boolean xState = false;
    boolean yState = false;
    boolean isShooting = false;
    boolean stateB1 = false;

    @Override
    public void runOpMode() {
        shooter = new Shooter(this);
        in = new Intake(this);
        sr = new Sorting(this);
        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.hwallOpen();
        sr.dwallClose();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b && !isShooting && !stateB1) {
                shooter.shoot();
                isShooting = true;
            } else if (gamepad1.b && isShooting && !stateB1) {
                shooter.shootStop();
                isShooting = false;
            }


            stateB1 = gamepad1.b;

            if (gamepad1.a) {
                sr.drumTele();
            } else {
                sr.drumStop();
            }

            if (gamepad1.y) {
                in.rotateIn();
            } else {
                in.rotateStop();
            }

            if (gamepad1.dpad_up) {
                sr.hwallClose();
            }
            if (gamepad1.dpad_down) {
                sr.hwallOpen();
            }
            if (gamepad1.dpad_left) {
                sr.dwallOpen();
            }
            if (gamepad1.dpad_right) {
                sr.dwallClose();
            }

            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition());
            telemetry.addData("Позиция стенки", sr.hwall.getController());
            telemetry.update();
        }
    }
//        sr.sortIntaker.interrupt();
//        sr.sortShooter.interrupt();
//        sr.sortMotorDriver.interrupt();
}


