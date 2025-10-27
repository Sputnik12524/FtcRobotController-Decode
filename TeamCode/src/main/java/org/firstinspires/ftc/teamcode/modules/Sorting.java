package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Sorting {
    private final ElapsedTime timer = new ElapsedTime();
    Shooter shooter = new Shooter();
    private final NormalizedColorSensor colorSensor;
    private final NormalizedColorSensor colorSensor2;
    private final NormalizedColorSensor colorSensor3;
    private final DcMotor drumMotor;
    private final DcMotor shootMotor;
    private final Servo wall;
    public Scan pos;
    //pos = shooter.pos;
    public static final float GAIN = 3F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    public static final double PULSES = 537.7;
    public static final double DEGREES = PULSES / 360;
    public static final double Ki = 0.001;
    public static final double Ks = 0.003;

    public enum Scan {LEFT, RIGHT, BETWEEN} // расположение зеленого арфтефакта

    enum Color {GREEN, PURPLE, NONE} //возможные цвета артефактов

    public static double SPEED = 0.5;

    public static double error;
    public static double errorS;
    public static final double OPEN_WALL = 0.57;
    public static final double CLOSE_WALL = 0.34;
    public static final double GREEN_MAX = 185;
    public static final double GREEN_MIN = 140;
    public static final double PURPLE_MAX = 245;
    public static final double PURPLE_MIN = 200;
    public SortMotorDriver sortMotorDriver = new SortMotorDriver();
    public SortIntake sortIntaker = new SortIntake();
    public SortSorting sortSorting = new SortSorting();
    public SortShooter sortShooter = new SortShooter();

    public Sorting(LinearOpMode opMode) {
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drum");
        this.shootMotor = opMode.hardwareMap.get(DcMotor.class, "shooter");
        this.wall = opMode.hardwareMap.get(Servo.class, "wall");
        this.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drumMotor.setDirection(DcMotor.Direction.REVERSE);
        this.colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        this.colorSensor.setGain(GAIN);
        this.colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        this.colorSensor2.setGain(GAIN);
        this.colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");
        this.colorSensor3.setGain(GAIN);
        this.drumMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public class SortMotorDriver extends Thread {

        @Override
        public void run() {
            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            timer.reset();

            while (!isInterrupted()) {
                intakingArtefacts();
                //sortingArtefacts(pos, artefact_pos(getColor()));
                shootingArtefacts(drumMotor.getCurrentPosition());
            }
        }
    }

    public class SortShooter extends Thread {

        @Override
        public void run() {
            while (!isInterrupted()) {
                shootingArtefacts(drumMotor.getCurrentPosition());
            }
        }
    }

    public class SortIntake extends Thread {

        @Override
        public void run() {
            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            timer.reset();

            while (!isInterrupted()) {
                intakingArtefacts();
            }
        }
    }

    public class SortSorting extends Thread {

        @Override
        public void run() {
            while (!isInterrupted()) {
                sortingArtefacts(pos, artefact_pos(getColor()));
            }
        }
    }

    public void intakingArtefacts() {
        int posOfBlades = 0; // это не константа
        while (getColor().get(2) == Color.NONE) { // 3 датчик нечего не видит
            posOfBlades += 120;

            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
                while (timer.milliseconds() < 350) {
                }
                while (drumMotor.getCurrentPosition() <= DEGREES * posOfBlades) {
                    error = posOfBlades - drumMotor.getCurrentPosition();
                    double power = error * Ki;
                    drumMotor.setPower(power);
                }
            }
            timer.reset();
        }
    }

    public void shootingArtefacts(double pos) {
        double drumPos = pos;
        drumPos += 120;
        switchingWall();

        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
            drumPos += 120;

            while (drumMotor.getCurrentPosition() <= drumPos){
                errorS = drumPos - drumMotor.getCurrentPosition();
                double power = errorS * Ks;
                drumMotor.setPower(power);
            }

        }
    }

    public void switchingWall() {
        if (wall.getPosition() == OPEN_WALL) {
            wall.setPosition(CLOSE_WALL);
        } else {
            wall.setPosition(OPEN_WALL);
        }
    }

    public void sortingArtefacts(Scan a, Scan b) {
        switch (a) {
            case LEFT: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
                            drumMotor.setPower(SPEED);
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case RIGHT: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
                            drumMotor.setPower(SPEED);
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case BETWEEN: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
                            drumMotor.setPower(SPEED);
                        break;
                }
                break;
            }

        }
    }

    public ArrayList<Color> getColor() {
        ArrayList<Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
        android.graphics.Color.colorToHSV(color1.toColor(), hsv);
        android.graphics.Color.colorToHSV(color2.toColor(), hsv2);
        android.graphics.Color.colorToHSV(color3.toColor(), hsv3);


        if (hsv[0] <= GREEN_MAX && hsv[0] >= GREEN_MIN) {
            colorSensors.add(Color.GREEN);
        } else if (hsv[0] <= PURPLE_MAX && hsv[0] >= PURPLE_MIN) {
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

    public Scan artefact_pos(ArrayList<Color> a) {
        Scan position = null;
        if (a.get(1) == Color.GREEN) position = Scan.LEFT;
        else if (a.get(0) == Color.GREEN) position = Scan.BETWEEN;
        else if (a.get(2) == Color.GREEN) position = Scan.RIGHT;

        return position;
    }

    public void drumTele(double power) {
        drumMotor.setPower(power);
    }
    public void shootTele(double power){shootMotor.setPower(power);}

    public void wallStarting() {
        wall.setPosition(OPEN_WALL);
    }

    public boolean isIntakeCompleted() {
        return getColor().get(2) != Color.NONE;
    }

    public boolean isSortingCompleted(Scan a, Scan b) {
        return a == b;
    }

    public boolean isShooterCompleted() {
        return getColor().get(1) == Color.NONE;
    }


}



