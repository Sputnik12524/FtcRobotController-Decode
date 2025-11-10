package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
    double i = 0;

    @Override
    public void runOpMode() {

        sr = new Sorting(this);

        sr.verticalWall.setPosition(0);
        sr.target = 0;// для проверки

        sr.regulatorSorting.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition() * sr.DEGREES);
            telemetry.addData("Позиция вертикальной стенки", sr.verticalWall.getPosition());


            telemetry.update();

            if (gamepad1.a) {
                i+= 0.05;
                sr.verticalWall.setPosition(i);
            }

            if(gamepad1.b && !bState){
                sr.target += 120;
            }

            if(gamepad1.x && !xState){
                sr.target += 10;
            }

            aState = gamepad1.a;
            bState = gamepad1.b;
            xState = gamepad1.x;
            yState = gamepad1.y;

        }
        sr.regulatorSorting.interrupt();

    }
}

