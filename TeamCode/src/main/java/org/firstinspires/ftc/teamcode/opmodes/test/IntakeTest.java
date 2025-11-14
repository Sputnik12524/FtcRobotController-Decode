package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Intake;

@TeleOp(name = "IntakeTest")
@Config
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.rotateIn();
            } else {
                intake.rotateStop();
            }
            if (gamepad1.b) {
                intake.rotateOut();
            } else {
                intake.rotateStop();
            }
        }
    }
}
