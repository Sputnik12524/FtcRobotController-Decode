package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.util.AimingMethod;

@TeleOp

public class TurretDumbTest extends LinearOpMode {
    public void runOpMode(){
        Turret tt = new Turret(this, new Limelight(this));
        double target = 0;

        tt.setAimMethod(AimingMethod.LOCALIZATION);
        tt.turretRegulator.start();

        waitForStart();
        while(opModeIsActive()) {
            tt.turnByTarget(target);

            if (gamepad1.a) {
                target += 1;
            } else if(gamepad1.b) {
                target -= 1;
            }
            telemetry.addData("target", target);
            telemetry.addData("position", tt.getCurrentPosOfTurret());
            telemetry.update();
        }
        tt.turretRegulator.interrupt();
    }
}
