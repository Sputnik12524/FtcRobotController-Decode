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


            sr.drumTele(gamepad1.left_stick_y / 10);


//            if (gamepad1.b && !bState) {
//                sr.sortIntaker.start();
//                while (!sr.isIntakeCompleted()) {
//                }
//                sr.sortIntaker.interrupt();
//            }
//
//            if (gamepad1.x && !xState) {
//                sr.sortMotorDriver.start();
//                while (!sr.isSortingCompleted(sr.artefact_pos(sr.getColor()), sr.pos)) { // под вопросом будет ли работать
//                }
//                sr.sortMotorDriver.interrupt();
//            }
//
//            if (gamepad1.y && !yState) {
//                sr.sortShooter.start();
//                while (!sr.isShooterCompleted()) {
//                }
//                sr.sortShooter.interrupt();
//            }


//            if (gamepad1.dpad_right) {
//                sr.drumTele(1);
//
//            }
//            if(gamepad2.a){
//                sr.turn40();
//            }
//            if (gamepad2.b){
//                sr.turn120();
//            }
//            if(gamepad2.x){
//                sr.simpleIntaking();
//            }
//            if (gamepad2.y){
//                sr.simpleShooting();
//            }

//            yState = gamepad2.y;
//            bState = gamepad1.b;
//            xState = gamepad2.x;
        }
//        sr.sortIntaker.interrupt();
//        sr.sortShooter.interrupt();
//        sr.sortMotorDriver.interrupt();
    }
}
