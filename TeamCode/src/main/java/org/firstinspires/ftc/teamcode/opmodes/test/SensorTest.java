package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "SensorTest", group = "Test")
public class SensorTest extends LinearOpMode {
    private com.qualcomm.robotcore.eventloop.opmode.LinearOpMode LinearOpMode;
    private org.firstinspires.ftc.teamcode.modules.Limelight Limelight;
    //private final DcMotor shooter;

    //color sensor 1 в порте 1;
    // 0 в порте 2;
    // 2 в порте 3

    @Override
    public void runOpMode() {
       Sorting sorting = new Sorting(this);
        telemetry.addLine("Начало");

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            if(gamepad1.a){
//
//                telemetry.addData("A",sorting.look_pos(sorting.getColor(), 0));
//
//            }
//            if(gamepad1.b){
//                telemetry.addData("B",sorting.look_pos(sorting.getColor(), 1));
//
//            }
//            if(gamepad1.x){
//                telemetry.addData("C",sorting.look_pos(sorting.getColor(), 2));
//
//            }
//            if(gamepad1.y){
//                telemetry.addData("Итог",sorting.artefact_pos(sorting.getColor()));
//            }
//            telemetry.update();
        }
    }
}
