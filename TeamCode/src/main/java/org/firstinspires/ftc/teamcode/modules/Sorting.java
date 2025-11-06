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
    private  ElapsedTime timer = new ElapsedTime();
    private  NormalizedColorSensor colorSensor1;
    private  NormalizedColorSensor colorSensor2;
    private  NormalizedColorSensor colorSensor3;
    public  DcMotor drumMotor;
    public  Servo hwall; // верхняяя сенка
    public  Servo dwall; // нижняя стенка
    public Scan pos;
    //pos = shooter.pos;
    public static  float GAIN = 3F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    public static double PULSES = 537.7;
    public static  double DEGREES = 360 / PULSES;
    public static  double k = 0.001;
    private static double target = 0;

    public enum Scan {LEFT, RIGHT, BETWEEN} // расположение зеленого арфтефакта

    enum Color {GREEN, PURPLE, NONE} //возможные цвета артефактов

    public static double SPEED = 0.5;
    
    public static double HOPEN_WALL = 0;
    public static double HCLOSE_WALL = 0.25;
    public static double DOPEN_WALL = 0;
    public static double DCLOSE_WALL = 0.65;
    public static double GREEN_MAX = 185;
    public static double GREEN_MIN = 140;
    public static double PURPLE_MAX = 245;
    public static double PURPLE_MIN = 200;

    public static double POWER = -0.4;
    public Regulator regulatorSorting = new Regulator();

    public Sorting(LinearOpMode opMode) {
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drum");
        this.hwall = opMode.hardwareMap.get(Servo.class, "hwall");
        this.dwall = opMode.hardwareMap.get(Servo.class, "dwall");
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
    
    public class Regulator extends Thread {
        @Override
        public void run() {
            while (!isInterrupted()) {
                double error = target - drumMotor.getCurrentPosition() * DEGREES;
                double power = error * k;
                drumMotor.setPower(power);
            }
        }
    }


    public void intakingArtefacts() {
        dwallOpen();
        while (getColor().get(2) == Color.NONE) { // 3 датчик нечего не видит
            
            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
                while (timer.milliseconds() < 350);
                target += 120;
            }
            timer.reset();
        }
    }

    public void shootingArtefacts() {
        turn40();
        dwallClose();
        hwallOpen();
        
        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
            while (timer.milliseconds() < 500) {
            }
            target -= 120;
        }
        timer.reset();
    }

    public void sortingArtefacts(Scan a, Scan b) {
        switch (a) {
            case LEFT: {
                switch (b) {
                    case LEFT:
                        target = 280;
                        break;
                    case RIGHT:
                        target = 160;
                        break;
                    case BETWEEN:
                        target = 40;
                        break;
                }
                break;
            }
            case RIGHT: {
                switch (b) {
                    case LEFT:
                        target = 40;
                        break;
                    case RIGHT:
                        target = 280;
                        break;
                    case BETWEEN:
                        target = 160;
                        break;
                }
                break;
            }
            case BETWEEN: {
                switch (b) {
                    case LEFT:
                        target = 40;
                        break;
                    case RIGHT:
                        target = 160;
                        break;
                    case BETWEEN:
                       target = 280;
                        break;
                }
                break;
            }

        }
    }

    public ArrayList<Color> getColor() {
        ArrayList<Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
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

    public boolean look_pos(ArrayList<Color> a, int b) { // для тестов
        return a.get(b) != Color.NONE;
    }

    public void drumStop() {
        drumMotor.setPower(0);
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
    
    public void turn40() {
        target += 40;
    }
    
    public void turnIn120() {
        target += 120;
    }

    public void turnOut120() {
        target -= 120;
    }

    

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

}



