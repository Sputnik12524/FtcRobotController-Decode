package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST ServoTest", group = "Test")
public class  ServoTest extends LinearOpMode {
    Sorting sr;

    boolean aState = false;
    boolean bState = false;
    boolean xState = false;
    boolean yState = false;

    double i =0;

    @Override
    public void runOpMode() {

        sr = new Sorting(this);

        sr.horizontalWallClose();
        sr.verticalWallClose();


        sr.target = 0;// для проверки

        sr.regulatorSorting.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition() * sr.DEGREES);
            telemetry.addData("Позиция вертикальной стенки", sr.verticalWall.getPosition());
            telemetry.addData("Позиция горизонтальной стенки", sr.horizontalWall.getPosition());
            telemetry.addData("Положение сортировки", sr.artefact_pos(sr.getColor()));


            telemetry.update();

            if (gamepad1.a && !aState) {
                i+= 0.05;
                sr.horizontalWall.setPosition(i);
            }

//            if (gamepad1.a && !aState) {
//                sr.target += 5;
//            }

            if(gamepad1.b && !bState){
                sr.target += 40;
            }

            if(gamepad1.x && !xState){
                sr.target += 120;
            }

            aState = gamepad1.a;
            bState = gamepad1.b;
            xState = gamepad1.x;
            yState = gamepad1.y;

        }
        sr.regulatorSorting.interrupt();

    }
}

