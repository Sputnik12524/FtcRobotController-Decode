package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Sorting {
    private final ElapsedTime timer = new ElapsedTime();
    Intake intake = new Intake();
    private final NormalizedColorSensor colorSensor;
    private final NormalizedColorSensor colorSensor2;
    private final NormalizedColorSensor colorSensor3;
    private final DcMotor drumMotor;
    private final Servo wall;
    public Scan pos;
    //pos = intake.pos;
    public static final float GAIN = 3F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    public static final double PULSES = 537.7;
    public static final double DEGREES = PULSES / 360;
    public static final double K = 0.001;

    enum Scan {LEFT, RIGHT, BETWEEN} // расположение зеленого арфтефакта

    enum Color {GREEN, PURPLE, NONE} //возможные цвета артефактов

    public static double SPEED = 0.5;

    public static double error;
    public static int posOfBlades = 0;
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
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drumMotor");
        this.wall = opMode.hardwareMap.get(Servo.class, "wall");
        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drumMotor.setDirection(DcMotor.Direction.REVERSE);
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensor.setGain(GAIN);
        colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        colorSensor2.setGain(GAIN);
        colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");
        colorSensor3.setGain(GAIN);
    }

    public class SortMotorDriver extends Thread {

        @Override
        public void run() {
            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            timer.reset();

            while (!isInterrupted()) {
                intaker();
                //drumTurner(pos, artefact_pos(getColor()));
                shooter(drumMotor.getCurrentPosition());
            }
        }
    }

    public class SortShooter extends Thread {

        @Override
        public void run() {
            while (!isInterrupted()) {
                shooter(drumMotor.getCurrentPosition());
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
                intaker();
            }
        }
    }

    public class SortSorting extends Thread {

        @Override
        public void run() {
            while (!isInterrupted()) {
                drumTurner(pos, artefact_pos(getColor()));
            }
        }
    }

    public void intaker() {
        while (getColor().get(2) == Color.NONE) { // 3 датчик нечего не видит
            posOfBlades += 120;
            error = posOfBlades - drumMotor.getCurrentPosition();
            double power = error * K;


            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
                while (timer.milliseconds() < 500) {
                }
                while (drumMotor.getCurrentPosition() <= DEGREES * posOfBlades)
                    drumMotor.setPower(power);
            }
            timer.reset();
        }
    }

    public void shooter(int pos) {
        int greenArtefactPos = pos;
        switchWall();
        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
            while (DEGREES * 120 <= greenArtefactPos) drumMotor.setPower(SPEED);
            greenArtefactPos += 120;
        }
    }

    public void switchWall() {
        if (wall.getPosition() == OPEN_WALL) {
            wall.setPosition(CLOSE_WALL);
        } else {
            wall.setPosition(OPEN_WALL);
        }
    }

    public void drumTurner(Scan a, Scan b) {
        switch (a) {
            case LEFT: {
                switch (b) {
                    case LEFT:
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case RIGHT: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case BETWEEN: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
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

    public void wallStart() {
        wall.setPosition(OPEN_WALL);
    }

    public boolean isIntakeCompleted() {
        if (getColor().get(2) == Color.NONE) {
            return false;
        } else return true;
    }


}



