package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Shooter;

@Disabled
@TeleOp(name = "TEST Set Velocities", group = "Test")
public class SetVelocityTest extends LinearOpMode {
    public static double SPEED = 360;
    public static double SPEED_LOW = 10;
    public static double SPEED_HIGH = 100;
    public static double SPEED_MIDDLE = 20;
    Shooter sh;

    boolean isShooting = false;
    boolean stateA1 = false;
    double p = 0;
    double i = 0;
    double d = 0;
    double f = 0;

    double target = 360;
    double targetUP = 360;
    double k = 1;
    @Override
    public void runOpMode() {

        sh = new Shooter(this);

        waitForStart();
        while (opModeIsActive()) {

            PIDFCoefficients pidfOrig = sh.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfOrigUP = sh.shooterTest.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


            double error = target - sh.shooter.getVelocity(AngleUnit.DEGREES);
            double power = error * k;
            sh.shooter.setVelocity(power, AngleUnit.DEGREES);

            double errorUp = targetUP - sh.shooterTest.getVelocity(AngleUnit.DEGREES);
            double powerUp = errorUp * k;
            sh.shooterTest.setVelocity(powerUp, AngleUnit.DEGREES);

            telemetry.addData("Shooter Velocity", sh.shooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("ShooterUP Velocity", sh.shooterTest.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (orig) UP", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrigUP.p, pidfOrigUP.i, pidfOrigUP.d, pidfOrigUP.f);
            telemetry.update();
        }
    }
}