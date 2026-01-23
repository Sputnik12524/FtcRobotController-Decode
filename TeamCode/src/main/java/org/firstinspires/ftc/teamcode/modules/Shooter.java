package org.firstinspires.ftc.teamcode.modules;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class Shooter {
    public final DcMotorEx shooterUpper;
    LinearOpMode opMode;
    public final DcMotorEx shooterLower;

    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;

    Follower follower;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(9.5, 0, 4, 15.2);
    private final double TPR = 28;
    public volatile int artifacts = 0;
    public int timers;
    public static double POWER = 1;
    public double velocityTarget = 0;
    public static double VELOCITY_FOR_LONG_THROW = 52.5;
    public static double VELOCITY_FOR_SHORT_THROW = 40;
    public static double ERROR = 2.5;
    public static double POS_COVER_OPEN = 0.5;
    public static double POS_COVER_CLOSE = 0.85;
    public static double POS_SHORT_THROW = 0.05;
    public static double POS_LONG_THROW = 0;
    boolean needShootPortion = false;

    public static double TIME_GATES_BETWEEN_SHOOT = 1000;
    public static double TIME_FOR_SET_VELOCITY = 2500;

    boolean InZone = true;
    public static boolean isTunnelOpen;


    public ContinuousShooter continuousShooter = new ContinuousShooter();
    public ShooterPortion portion = new ShooterPortion();
    private final ElapsedTime timerSh = new ElapsedTime();
    private Pose currentPose;

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");


        shooterUpper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterUpper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterLower.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(shooterUpper, MOTOR_VELO_PID_SHOOTERS);
        setPIDFCoefficients(shooterLower, MOTOR_VELO_PID_SHOOTERS);
    }

//    public Shooter(LinearOpMode opMode, Follower follower) {
//        this.follower = follower;
//        this.opMode = opMode;
//        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
//        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
//        cover = opMode.hardwareMap.get(Servo.class, "cover");
//        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");
//      //  currentPose = follower.getPose();
//
//        shooterUpper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooterUpper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooterLower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooterLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        shooterLower.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
//        setPIDFCoefficients(shooterUpper, MOTOR_VELO_PID_SHOOTERS);
//        setPIDFCoefficients(shooterLower, MOTOR_VELO_PID_SHOOTERS);
//    }

    public void shootByVelocity() {
        shooterUpper.setVelocity(velocityTarget);
        shooterLower.setVelocity(velocityTarget);
    }

    public void setVelocityTarget(double targetInRPS) {
        velocityTarget = targetInRPS * TPR;
    }

    public void shootByPower() {
        shooterUpper.setPower(POWER);
        shooterLower.setPower(POWER);
    }

    public void shootStop() {
        shooterUpper.setVelocity(0);
        shooterLower.setVelocity(0);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void waitForShoot() { // no test
        for (int i = 0; i < 5; i++) {
            opMode.sleep(2000);
            openTunnel();
            isTunnelOpen = true;
            opMode.sleep(200);
            closeTunnel();
            isTunnelOpen = false;
        }

    }

    public void setShortThrowMode() {
        angleAdjuster.setPosition(POS_SHORT_THROW);
    }

    public void setLongThrowMode() {
        angleAdjuster.setPosition(POS_LONG_THROW);
    }

    public void openTunnel() {
        cover.setPosition(POS_COVER_OPEN);
    }

    public void closeTunnel() {
        cover.setPosition(POS_COVER_CLOSE);
    }

    public double getAngleAdjusterPos() {
        return angleAdjuster.getPosition();
    }

 //   public boolean ifInLaunchZoneGoal() {
        //currentPose = follower.getPose();
//        if (follower.getPose().getY() >= Math.abs(follower.getPose().getX() - 72) + 72) {
//            return true;
//        } else {
//            return false;
//        }
   // }

//    public boolean ifInLaunchZoneHuman() {
//        currentPose = follower.getPose();
//        if (follower.getPose().getY() <= -Math.abs(follower.getPose().getX() - 72) + 24) {
//            return true;
//        } else {
//            return false;
//        }
//    }

//    public void shootingAllowed() {
//        if (ifInLaunchZoneGoal() || ifInLaunchZoneHuman()) {
//            openTunnel();
//        } else {
//            closeTunnel();
//        }
//    }


    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                shootByVelocity();
            }
        }
    }

    public class ShooterPortion extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                if (needShootPortion) {
                    timer.reset();
                    closeTunnel();
                    setLongThrowMode();
                    setVelocityTarget(VELOCITY_FOR_LONG_THROW);
                    shootByVelocity();
                    while (timer.milliseconds() <= TIME_FOR_SET_VELOCITY) ;
                    for (int i = 0; i < 3; i += 1) {
                        timer.reset();
                        openTunnel();
                        while (timer.milliseconds() <= TIME_GATES_BETWEEN_SHOOT) ;
                        timer.reset();
                        closeTunnel();
                        while (timer.milliseconds() <= TIME_GATES_BETWEEN_SHOOT) ;
                    }
                    needShootPortion = false;
                }
            }
        }
    }

    public void needShootPortion() {
        this.needShootPortion = true;
    }

    public void updateCalculator() {
        if (shooterUpper.getVelocity() < velocityTarget) {
            artifacts++;
            timers = 1000;
        }
        timers = 50;
    }

    public double getPower() {
        return shooterUpper.getPower();
    }

    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }


}
