package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;

@TeleOp(name = "setVelocityTest", group = "Test")
public class SetVelocityTest extends LinearOpMode {
    public static double SPEED = 360;
    public static double SPEED_LOW = 1;
    public static double SPEED_HIGH = 100;
    public static double SPEED_MIDDLE = 20;
    Shooter sh;

    boolean isShooting = false;
    boolean stateA1 = false;

    @Override
    public void runOpMode() {

        sh = new Shooter(this);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a && !isShooting && !stateA1) {
                sh.setVelocity(SPEED_LOW);
                isShooting = true;
            } else if (gamepad1.a && isShooting && !stateA1) {
                sh.setVelocity(0);
                isShooting = false;
            }
            stateA1 = gamepad1.a;

            telemetry.addData("Shooter Velocity", sh.shooter.getVelocity());
            telemetry.addData("ShooterUP Velocity", sh.shooterUp.getVelocity());
            telemetry.update();
        }
    }
}