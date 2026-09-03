package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

@TeleOp(name = "TeleOpFirstSeptember", group = "0")
@Config
public class TeleOpFirstSeptember extends LinearOpMode {

    DriveTrain dt;
    Shooter sh;
    Intake in;
    Turret tt;

    @Override
    public void runOpMode() {

        dt = new DriveTrain(this);
        sh = new Shooter(this);
        in = new Intake(this);
        tt = new Turret(this);


        sh.closeTunnel();
        tt.turnByTarget(0);

        waitForStart();

        while (opModeIsActive()) {
            double main = -gamepad1.left_stick_y;
            double side = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

            DriveTrain.multiplier = 0.5;

            /// DriveTrain
            dt.setPower(main, side, rotate);


            /// Intake
            if (gamepad1.aWasPressed()) {
                in.rotateIn();
            } else if (gamepad1.bWasPressed()) {
                in.rotateOut();
            } else {
                in.rotateStop();
            }


            /// Shooter
            if (gamepad1.yWasPressed()) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.shootByVelocity();
            } else {
                sh.setVelocityTarget(0);
            }


            if (gamepad1.dpadUpWasPressed()) {
                sh.openTunnel();
            } else if (gamepad1.dpadDownWasPressed()) {
                sh.closeTunnel();
            }

        }
    }
}