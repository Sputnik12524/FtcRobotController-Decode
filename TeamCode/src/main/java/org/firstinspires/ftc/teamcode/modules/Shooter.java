package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.opmodes.tele.TeleOpRoadRunnerV2;

import java.util.ArrayList;

@Config
public class Shooter {
    public final DcMotorEx shooterUpper;
    LinearOpMode opMode;
    public final DcMotorEx shooterLower;

    enum Color {GREEN, PURPLE, NONE}

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    private float[] hsv1 = new float[2];
    private float[] hsv2 = new float[2];
    private float[] hsv3 = new float[2];
    public static double GREEN_MAX = 175;
    public static double GREEN_MIN = 115;
    public static double PURPLE_MAX = 245;
    public static double PURPLE_MIN = 210;
    public final float GAIN = 2.4f;
    public boolean canShoot = false; //


    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;

    Follower follower;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(9.5, 0, 4, 15.2);
    private final double TPR = 28;
    public short artifacts = 0;
    public short artifactsIn = 0;
    public short artifactsNow = 0;
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
    public static double TIME_BETWEEN_SHOOT = 300;
    boolean needShootPortion = false;
    public boolean isShooting = false;
    public boolean detected = false;
    int timerses = 0;

    public static double TIME_GATES_BETWEEN_SHOOT = 1000;
    public static double TIME_FOR_SET_VELOCITY = 2500;
    private final ElapsedTime timer = new ElapsedTime();

    boolean InZone = true;
    public static boolean isTunnelOpen;

    enum states {DEFAULT, INIT, SHOOT, UPDATE, RESTART, START}

    states state = states.INIT;

    public ContinuousShooter continuousShooter = new ContinuousShooter();
    //public ShooterPortion portion = new ShooterPortion();
    private final ElapsedTime timerSh = new ElapsedTime();
    private Pose currentPose;

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        shooterUpper = opMode.hardwareMap.get(DcMotorEx.class, "shooterUpper");
        shooterLower = opMode.hardwareMap.get(DcMotorEx.class, "shooterLower");
        cover = opMode.hardwareMap.get(Servo.class, "cover");
        angleAdjuster = opMode.hardwareMap.get(Servo.class, "angleAdjuster");
        colorSensor1 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor1.setGain(GAIN);
        colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor2.setGain(GAIN);
        colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");
        colorSensor3.setGain(GAIN);


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
        isShooting = false;
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
        isShooting = true;

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

    public void setMode(double pos) {
        angleAdjuster.setPosition(pos);
    }

    public boolean threeArtefactsShooting() {

        updateCalculator(velocityTarget);
        if (detected) {

            for (int i = 0; i < 3; i++) {
                timer.reset();
                while (timer.milliseconds() < TIME_BETWEEN_SHOOT) {
                }
                setMode(angleAdjuster.getPosition() - 0.007);
            }
            return false;
        }
        else return true;

    }

//        switch (state){
//            case INIT:
//                if(isShooting && inZone && canShoot){
//                    transit(states.SHOOT);
//                }
//            case SHOOT:
//                openTunnel();
//                updateCalculator(VELOCITY);
//                if(detected){
//
//                }
//
//        }
//    }


    public void autoStupidSetVelocityAndAngle(double y) {
        if (y < 48) {
            setLongThrowMode();
            setVelocityTarget(VELOCITY_FOR_LONG_THROW);
        } else if (y > 84) {
            setShortThrowMode();
            setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
        }
    }

    public ArrayList<Color> getColor() {
        ArrayList<Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
        android.graphics.Color.colorToHSV(color1.toColor(), hsv1);
        android.graphics.Color.colorToHSV(color2.toColor(), hsv2);
        android.graphics.Color.colorToHSV(color3.toColor(), hsv3);


        if (hsv1[0] <= GREEN_MAX && hsv1[0] >= GREEN_MIN) {
            colorSensors.add(Color.GREEN);
        } else if (hsv1[0] <= PURPLE_MAX && hsv1[0] >= PURPLE_MIN) {
            colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);


        if (hsv2[0] <= GREEN_MAX && hsv2[0] >= GREEN_MIN) {
            colorSensors.add(Color.GREEN);
        } else if (hsv2[0] <= PURPLE_MAX && hsv2[0] >= PURPLE_MIN) {
            colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);


        if (hsv3[0] <= GREEN_MAX && hsv3[0] >= GREEN_MIN) {
            colorSensors.add(Color.GREEN);
        } else if (hsv3[0] <= PURPLE_MAX && hsv3[0] >= PURPLE_MIN) {
            colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);

        return colorSensors;
    }

    public boolean isEmpty() {
        for (int i = 0; i < 3; ++i) {
            if (getColor().get(i) != Color.NONE) return true;
        }
        return false;
    }

    public int artefactsIn() {
        artifactsIn = 0;
        for (int i = 0; i < 3; i++) {
            if (getColor().get(i) != Color.NONE) artifactsIn++;
        }
        return artifactsIn;
    }


    public void updateCalculator(double RPS) {
        if (isShoot(RPS) && detected) {
            detected = false;
            artifacts++;
        }
        if(isBack(RPS)) detected = true;
    }

    public boolean isShoot(double RPS) {
        return getVelocityRPS() < RPS - 4;
    }

    public double getVelocityRPS() {
        return shooterUpper.getVelocity() / TPR;
    }

    public double getVelocityTPS() {
        return shooterUpper.getVelocity();
    }

    public void sleeping(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public boolean isBack(double RPS) {
        return getVelocityRPS() >= (RPS - 1); //погрешность подобрать
    }

//    public void transit(states state) {
//        timer.reset();
//        this.state = state;
//    }

}

