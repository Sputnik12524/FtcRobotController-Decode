package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter { //sh

    public final DcMotorEx shooter;
    private final VoltageSensor batteryVoltageSensor;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTER_OLD = new PIDFCoefficients(30, 0, 30, 27);
    public static PIDFCoefficients MOTOR_VELO_PID_TURRET = new PIDFCoefficients(0, 0, 0, 0);
    private final double TPR = 28;

    public static double POWER = 1;
    public static double VELOCITY = 0;
    public static double VELO_HUMAN = 45;
    public static double VELO_GOAL = 37;
    public static double ERROR = 3;
    boolean StateA = false;
    boolean isShooting = false;

    public ContinuousShooter continuousShooter = new ContinuousShooter();

    public Shooter(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(shooter, MOTOR_VELO_PID_SHOOTER_OLD);

    }

    public void shootByVelocity(double RPS) {
        shooter.setVelocity(RPS * TPR);
    }

    public void shootByPower(double POWER) {
        shooter.setPower(POWER);
    }

    public void setPower(double POWER) {
        shooter.setPower(POWER);
    }

    public void shootStop() {
        //shooter.setVelocity(0);
        shooter.setPower(0);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void waitForShoot(double velocity) {
        setVelocityAuto(velocity);
        while (getVelocityRPS() >= VELO_GOAL + ERROR || getVelocityRPS() <= VELO_GOAL - ERROR) {}
        //тут открытие крышки
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                timer.reset();
                shooter.setVelocity(VELOCITY);
                while (timer.milliseconds() < 23000) ;
                shooter.setVelocity(0);
            }
        }
    }

    public void setVelocityAuto(double RPS) { //For ContinuousShooter
        VELOCITY = RPS * TPR;
    }

    public double getPower() {
        return shooter.getPower();
    }

    public double getVelocityRPS() {
        return shooter.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooter.getVelocity();
    }

}
