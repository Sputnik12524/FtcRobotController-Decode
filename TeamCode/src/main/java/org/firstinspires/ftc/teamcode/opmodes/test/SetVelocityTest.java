package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "setVelocityTest", group = "Test")
public class SetVelocityTest extends LinearOpMode {
    Shooter sh;

    @Override
    public void runOpMode() {
        sh = new Shooter(this);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a){
                sh.setVelocity();
            } else {
                sh.shootStop();
            }
            telemetry.addData("Shooter Velocity", sh.shooter.getVelocity());
        }
    }
}