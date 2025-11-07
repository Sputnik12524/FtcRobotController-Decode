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

    @Override
    public void runOpMode() {

        sr = new Sorting(this);


        sr.target = 0;// для проверки

        sr.regulatorSorting.start();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Позиция сортировки", sr.drumMotor.getCurrentPosition() * sr.DEGREES);

            telemetry.update();

            if (gamepad1.a) {
                sr.target += 40;
            }

            if(gamepad1.b){
                sr.target += 120;
            }

            if(gamepad1.x){
                sr.target += 10;
            }

        }
        sr.regulatorSorting.interrupt();

    }
}

