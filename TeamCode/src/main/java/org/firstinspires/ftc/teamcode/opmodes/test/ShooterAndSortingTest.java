package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {

    Sorting sr;

    //Shooter st;
    boolean aState = false;
    boolean bState = false;
    boolean xState = false;
    boolean yState = false;
    boolean a2State = false;
    boolean b2State = false;

    @Override
    public void runOpMode() {
        sr = new Sorting(this);

        sr.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sr.pos = Sorting.Scan.RIGHT; // для проверки
        sr.horizontalWallClose();
        sr.verticalWallClose();
        sr.regulatorSorting.start();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition() * sr.DEGREES);
            telemetry.addData("Положение сортировки", sr.artefact_pos(sr.getColor()));
            telemetry.addData("Цвета в сортировке", sr.printGetColor(sr.getColor()));
//            telemetry.addData("Позиция горизонтальной стенки", sr.horizontalWall.getPosition());
//            telemetry.addData("Позиция вертикальной стенки", sr.verticalWall.getPosition());

            telemetry.update();

            if (gamepad1.a && !aState) {
                sr.intakingArtefacts();
            }

            if (gamepad1.b && !bState) {
                sr.sortingArtefacts(sr.artefact_pos(sr.getColor()), sr.pos);
            }

            if (gamepad1.x && !xState) {
                sr.shootingArtefacts();
            }

            if (gamepad1.y && !yState) {
                sr.turn40();
            }

            if (gamepad2.a && a2State) {
                sr.turnIn120();
            }

            if (gamepad2.b && b2State) {
                sr.turnOut120();
            }


            if (gamepad1.dpad_up) {
                sr.horizontalWallOpen();
            }

            if (gamepad1.dpad_down) {
                sr.horizontalWallClose();
            }
            if (gamepad1.dpad_left) {
                sr.verticalWallOpen();
            }
            if (gamepad1.dpad_right) {
                sr.verticalWallClose();
            }


            aState = gamepad1.a;
            bState = gamepad1.b;
            xState = gamepad1.x;
            yState = gamepad1.y;
            a2State = gamepad2.a;
            b2State = gamepad2.b;
        }
        sr.regulatorSorting.interrupt();

    }
}

