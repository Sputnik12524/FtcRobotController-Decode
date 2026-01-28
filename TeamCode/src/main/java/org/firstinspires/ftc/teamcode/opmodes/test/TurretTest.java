package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Turret;

@Config
@TeleOp(name = "TEST Turret", group = "1")
public class TurretTest extends LinearOpMode {

    Turret tr;

    public static double TARGET = 0;

    @Override
    public void runOpMode() {
        tr = new Turret(this);
        tr.turretRegulator.start();
        waitForStart();
        while (opModeIsActive()) {

            tr.turnByTarget(TARGET);

            telemetry.addData("Magneting?", tr.isMagneting());
            telemetry.addData("speed", tr.turret.getVelocity());
            telemetry.addData("target", tr.target);
            telemetry.addData("error", tr.error);
            telemetry.addData("Pos", tr.getCurrentPosOfTurret());
            telemetry.update();
        }
        tr.turretRegulator.interrupt();
    }
}
