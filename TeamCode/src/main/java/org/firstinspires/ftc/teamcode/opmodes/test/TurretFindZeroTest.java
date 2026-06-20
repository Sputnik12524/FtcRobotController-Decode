package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.util.AimingMethod;

@TeleOp

public class TurretFindZeroTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret tt = new Turret(this, new Limelight(this));

        double target = 0;
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        tt.turretRegulator.start();

        waitForStart();

        while(opModeIsActive()){
            if(!tt.isMagneting()) {
                target += 0.1;
                tt.turnByTarget(target);

            } else if(tt.isMagneting()){
                target = 0;
                    tt.turnStopByPower();
            }

            telemetry.addData("Angle", tt.getCurrentPosOfTurret());
            telemetry.addData("target", target);
            telemetry.addData("Magneting", tt.isMagneting());
            telemetry.update();
        }
        tt.turretRegulator.interrupt();
    }
}
