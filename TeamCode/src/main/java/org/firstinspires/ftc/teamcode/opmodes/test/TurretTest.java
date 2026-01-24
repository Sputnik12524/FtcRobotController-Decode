package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Turret;

@Config
@TeleOp(name = "TEST Turret", group = "1")
public class TurretTest extends LinearOpMode {

    Turret tr;

    @Override
    public void runOpMode() {
        tr = new Turret(this);
        tr.turretRegulator.start();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                tr.turnRightByTarget();
            } else if (gamepad1.left_bumper) {
                tr.turnLeftByTarget();
            }

            telemetry.addData("Magneting?", tr.isMagneting());
            telemetry.addData("speed", tr.turret.getVelocity());
            telemetry.addData("t", tr.target);
            telemetry.addData("error", tr.error);
            telemetry.update();
        }
        tr.turretRegulator.interrupt();
    }
}
