package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Sorting {
    private final ElapsedTime timer = new ElapsedTime();
    private final NormalizedColorSensor colorSensor1;
    private final NormalizedColorSensor colorSensor2;
    private final NormalizedColorSensor colorSensor3;
    public final DcMotor drumMotor;
    //private final DcMotor shootMotor;
    public final Servo hwall; // верхняяя сенка
    public final Servo dwall; // нижняя стенка
    public Scan pos;
    //pos = shooter.pos;
    public static final float GAIN = 3F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    public static final double PULSES = 537.7;
    public static final double DEGREES = 360 / PULSES;
    public static final double Ki = 0.001;
    public static final double Ks = 0.003;
    private double k = 0.001;
    private static double target = 0;


    public enum Scan {LEFT, RIGHT, BETWEEN, NONE} // расположение зеленого арфтефакта

    enum Color {GREEN, PURPLE, NONE} //возможные цвета артефактов

    public static double SPEED = 0.5;

    public static double errorS;

    public static double HOPEN_WALL = 0;
    public static double HCLOSE_WALL = 0.25;
    public static double DOPEN_WALL = 0;
    public static double DCLOSE_WALL = 0.65;
    public static double GREEN_MAX = 185;
    public static double GREEN_MIN = 140;
    public static double PURPLE_MAX = 245;
    public static double PURPLE_MIN = 200;

    public static double POWER = -0.4;

    public Turn120 turn120 = new Turn120();
//    public SortMotorDriver sortMotorDriver = new SortMotorDriver();
//    public SortIntake sortIntaker = new SortIntake();
//    public SortSorting sortSorting = new SortSorting();
//    public SortShooter sortShooter = new SortShooter();

    public Sorting(LinearOpMode opMode) {
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drum");
        this.hwall = opMode.hardwareMap.get(Servo.class, "horizontalWall");
        this.dwall = opMode.hardwareMap.get(Servo.class, "verticalWall");
        this.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drumMotor.setDirection(DcMotor.Direction.REVERSE);
        this.colorSensor1 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        this.colorSensor1.setGain(GAIN);
        this.colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        this.colorSensor2.setGain(GAIN);
        this.colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");
        this.colorSensor3.setGain(GAIN);
    }


//    public class SortMotorDriver extends Thread {
//
//        @Override
//        public void run() {
//            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            timer.reset();
//
//            while (!isInterrupted()) {
//                intakingArtefacts();
//                //sortingArtefacts(pos, artefact_pos(getColor()));
//                shootingArtefacts(drumMotor.getCurrentPosition());
//            }
//        }
//    }
//
//    public class SortShooter extends Thread {
//
//        @Override
//        public void run() {
//            while (!isInterrupted()) {
//                shootingArtefacts(drumMotor.getCurrentPosition());
//            }
//        }
//    }
//
//    public class SortIntake extends Thread {
//
//        @Override
//        public void run() {
//            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            timer.reset();
//
//            while (!isInterrupted()) {
//                intakingArtefacts();
//            }
//        }
//    }
//
//    public class SortSorting extends Thread {
//
//        @Override
//        public void run() {
//            while (!isInterrupted()) {
//                sortingArtefacts(pos, artefact_pos(getColor()));
//            }
//        }
//    }
//
//
//    public void intakingArtefacts() {
//        dwallOpen();
//        int posOfBlades = 0; // это не константа
//        while (getColor().get(2) == Color.NONE) { // 3 датчик нечего не видит
//            posOfBlades += 120;
//
//            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
//                while (timer.milliseconds() < 350) {
//                }
//                while (drumMotor.getCurrentPosition() <= DEGREES * posOfBlades) {
//                    error = posOfBlades - drumMotor.getCurrentPosition();
//                    double power = error * Ki;
//                    drumMotor.setPower(power);
//                }
//            }
//            timer.reset();
//        }
//    }
//
//    public void shootingArtefacts(double pos) {
//        turn40();
//        double drumPos = pos;
//        dwallClose();
//        hwallOpen();
//
//
//        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
//            drumPos += 120;
//
//            while (drumMotor.getCurrentPosition() <= drumPos) {
//                errorS = drumPos - drumMotor.getCurrentPosition();
//                double power = errorS * Ks;
//                drumMotor.setPower(power);
//            }
//
//        }
//    }

    //    public void sortingArtefacts(Scan a, Scan b) {
//        switch (a) {
//            case LEFT: {
//                switch (b) {
//                    case LEFT:
//                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
//                            drumMotor.setPower(SPEED);
//                        break;
//                    case RIGHT:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                    case BETWEEN:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                }
//                break;
//            }
//            case RIGHT: {
//                switch (b) {
//                    case LEFT:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                    case RIGHT:
//                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
//                            drumMotor.setPower(SPEED);
//                        break;
//                    case BETWEEN:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                }
//                break;
//            }
//            case BETWEEN: {
//                switch (b) {
//                    case LEFT:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 40)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                    case RIGHT:
//                        while (drumMotor.getCurrentPosition() > DEGREES * 160)
//                            drumMotor.setPower(-SPEED);
//                        break;
//                    case BETWEEN:
//                        while (drumMotor.getCurrentPosition() < DEGREES * 280)
//                            drumMotor.setPower(SPEED);
//                        break;
//                }
//                break;
//            }
//
//        }
//    }
//
//    public ArrayList<Color> getColor() {
//        ArrayList<Color> colorSensors = new ArrayList<>();
//
//        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
//        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
//        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
//        android.graphics.Color.colorToHSV(color1.toColor(), hsv);
//        android.graphics.Color.colorToHSV(color2.toColor(), hsv2);
//        android.graphics.Color.colorToHSV(color3.toColor(), hsv3);
//
//
//        if (hsv[0] <= GREEN_MAX && hsv[0] >= GREEN_MIN) {
//            colorSensors.add(Color.GREEN);
//        } else if (hsv[0] <= PURPLE_MAX && hsv[0] >= PURPLE_MIN) {
//            colorSensors.add(Color.PURPLE);
//        } else colorSensors.add(Color.NONE);
//
//
//        if (hsv2[0] <= GREEN_MAX && hsv2[0] >= GREEN_MIN) {
//            colorSensors.add(Color.GREEN);
//        } else if (hsv2[0] <= PURPLE_MAX && hsv2[0] >= PURPLE_MIN) {
//            colorSensors.add(Color.PURPLE);
//        } else colorSensors.add(Color.NONE);
//
//
//        if (hsv3[0] <= GREEN_MAX && hsv3[0] >= GREEN_MIN) {
//            colorSensors.add(Color.GREEN);
//        } else if (hsv3[0] <= PURPLE_MAX && hsv3[0] >= PURPLE_MIN) {
//            colorSensors.add(Color.PURPLE);
//        } else colorSensors.add(Color.NONE);
//
//        return colorSensors;
//    }
//
//    public Scan artefact_pos(ArrayList<Color> a) {
//        Scan position = null;
//        if (a.get(1) == Color.GREEN) position = Scan.LEFT;
//        else if (a.get(0) == Color.GREEN) position = Scan.BETWEEN;
//        else if (a.get(2) == Color.GREEN) position = Scan.RIGHT;
//
//        return position;
//    }
//
//    public boolean look_pos(ArrayList<Color> a, int b) { // для тестов
//        return a.get(b) != Color.NONE;
//    }
    public void drumStop() {
        drumMotor.setPower(0);
    }

    public void drumTele() {
        drumMotor.setPower(POWER);
    }

    public void hwallClose() {
        hwall.setPosition(HCLOSE_WALL);
    }

    public void dwallClose() {
        dwall.setPosition(DCLOSE_WALL);
    }

    public void hwallOpen() {
        hwall.setPosition(HOPEN_WALL);
    }

    public void dwallOpen() {
        dwall.setPosition(DOPEN_WALL);
    }

    public void setDWallPos(double a) {
        dwall.setPosition(a);
    }

    public void autoDrumTurningIn(double POWER) {
        while (timer.seconds() < 5) {
            drumMotor.setPower(-POWER); //изменить
        }
        timer.reset();
    }

    public void autoDrumTurningOut(double POWER) {
        while (timer.seconds() < 5) {
            drumMotor.setPower(POWER); //изменить
        }
        timer.reset();
    }

    public void setTarget120() {
        target = target + 120;
    }


    public class Turn120 extends Thread {
        @Override
        public void run() {
            while (!isInterrupted()) {
                double error = target - drumMotor.getCurrentPosition() * DEGREES;
                double power = error * k;
                drumMotor.setPower(power);
            }
        }
    }
    /* они же крутят долго, чтобы шарики вылетели все, а если через такое делать то оно будет только по ступенькам двигаться и такого эффекта не получится
    ну гг
    гг х2
    там оно же еще будет постепенно крутиться
    типа с нарастабщей скоростью
    поэтому фигли оно разгонится
    убраться в техзоне
     */


//    public boolean isIntakeCompleted() {
//        return getColor().get(2) != Color.NONE;
//    }

//    public boolean isSortingCompleted(Scan a, Scan b) {
//        return a == b;
//    }

//    public boolean isShooterCompleted() {
//        return getColor().get(1) == Color.NONE;
//    }


//    public void turn40() {
//        int bladesPos = drumMotor.getCurrentPosition();
//        bladesPos -= 40;
//        while (drumMotor.getCurrentPosition() > bladesPos * DEGREES) {
//            drumMotor.setPower(-SPEED);
//        }
//    }


//    public void turn120() {
//        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int bladesPos = -120;
//        while (drumMotor.getCurrentPosition() > bladesPos * DEGREES) {
//            drumMotor.setPower(-SPEED);
//        }
//    }

//    public void simpleShooting() {
//        dwallClose();
//        hwallOpen();
//        turn40();
//        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int bladesPos = -120;
//        for (int i = 0; i < 3; i++) {
//            while (drumMotor.getCurrentPosition() > bladesPos * DEGREES) drumMotor.setPower(-SPEED);
//            bladesPos -= 120;
//        }
//    }

    //    public void simpleIntaking() {
//
//        dwallOpen();
//
//        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int bladesPos = -120;
//
//        for (int i = 0; i < 3; i++) {
//            while (drumMotor.getCurrentPosition() > bladesPos * DEGREES) drumMotor.setPower(-SPEED);
//            bladesPos -= 120;
//        }
//    }
    public Scan aprilTagToScan(int id) {
        Scan res = Scan.NONE;
        if (id == 21) {
            res = Scan.LEFT;
        } else if (id == 22) {
            res = Scan.BETWEEN;
        } else if (id == 23) {
            res = Scan.RIGHT;
        }
        return res;
    }
}



