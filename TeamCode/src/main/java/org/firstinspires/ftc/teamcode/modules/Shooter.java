package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Shooter {

    public final DcMotorEx shooterUpper;
    LinearOpMode opMode;
    public final DcMotorEx shooterLower;
    enum Color{GREEN, PURPLE, NONE}

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
    public boolean canShoot = false;



    public final Servo angleAdjuster;
    public final Servo cover;
    private final VoltageSensor batteryVoltageSensor;
    public static PIDFCoefficients MOTOR_VELO_PID_SHOOTERS = new PIDFCoefficients(20, 0, 30, 14.7);
    private final double TPR = 28;
    public  short artifacts = 0;
    public short artifactsIn = 0;
    public short artifactsNow = 0;
    public int timers;
    public static double POWER = 1;
    public double velocityTarget = 0;
    public static double VELOCITY_FOR_LONG_THROW = 52.5;
    public static double VELOCITY_FOR_MIDDLE_THROW = 48; //подобрать
    public static double VELOCITY_FOR_SHORT_THROW = 43;
    public  double VELOCITY = 0;
    public static double ERROR = 2.5;
    public static double POS_COVER_OPEN = 0.72;
    public static double POS_COVER_CLOSE = 1;
    public static double POS_SHORT_THROW = 0.75;
    public static double POS_LONG_THROW = 1;
    boolean needShootPortion = false;
    public boolean isShooting = false;

    public static double TIME_GATES_BETWEEN_SHOOT = 1000;
    public static double TIME_FOR_SET_VELOCITY = 2500;


    public ContinuousShooter continuousShooter = new ContinuousShooter();
    private final ElapsedTime timerSh = new ElapsedTime();

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

    public void shootByVelocity() {
        shooterUpper.setVelocity(velocityTarget);
        shooterLower.setVelocity(velocityTarget);
        isShooting = true;
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

    public void setMode(double pos){
        angleAdjuster.setPosition(pos);
    }

    public void autoStupidSetVelocityAndAngle(double y) {
        if (y < 48) {
            setLongThrowMode();
            setVelocityTarget(VELOCITY_FOR_LONG_THROW);
        } else if (y > 84) {
            setShortThrowMode();
            setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
        } else {
            setShortThrowMode();
            setVelocityTarget(VELOCITY_FOR_MIDDLE_THROW);
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

    public String isEmpty(){
        for(int i = 0; i < 3; ++i){
            if(getColor().get(i) != Color.NONE) return "Пустой";
        }
        return "Не пустой";
    }
    public int artefactsIn(){
        artifactsIn = 0;
        for(int i = 0; i < 3; i ++){
            if(getColor().get(i) != Color.NONE) artifactsIn++;
        }
        return artifactsIn;
    }

    public class ContinuousShooter extends Thread {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void run() {
            if (!isInterrupted()) {
                shootByVelocity();
            }
        }
    }

    public void needShootPortion() {
        this.needShootPortion = true;
    }

    public void updateCalculator(double RPS) {
        if(artifactsNow == 3) artifactsNow = 0;

        if (isShoot(RPS) && RPS != 0) {
            artifacts++;
            artifactsNow++;
        } else {
            timers = 50;
        }
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

}
