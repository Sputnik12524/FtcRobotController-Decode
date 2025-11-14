package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Sorting {
    private ElapsedTime timer = new ElapsedTime();

    //Shooter shooter;

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    public DcMotor drumMotor;
    public Servo horizontalWall; // верхняяя сенка
    public Servo verticalWall; // нижняя стенка
    public Scan pos;
    //pos = shooter.pos;
    public static float GAIN = 2.4F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    public static double PULSES = 537.7;
    public static double DEGREES = 360 / PULSES;
    public static double k = 0.002;
    public static double target = 0;


    public enum Scan {LEFT, RIGHT, BETWEEN, NONE} // расположение зеленого арфтефакта

    enum Color {GREEN, PURPLE, NONE} //возможные цвета артефактов

    public static double HOPEN_WALL = 0.65;
    public static double HCLOSE_WALL = 0;
    public static double VOPEN_WALL = 0.25;
    public static double VCLOSE_WALL = 0;
    public static double GREEN_MAX = 175;
    public static double GREEN_MIN = 115;
    public static double PURPLE_MAX = 245;
    public static double PURPLE_MIN = 210;
    public Regulator regulatorSorting = new Regulator();

    public Sorting(LinearOpMode opMode) { // Shooter shooter
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drum");
        this.horizontalWall = opMode.hardwareMap.get(Servo.class, "horizontalWall");
        this.verticalWall = opMode.hardwareMap.get(Servo.class, "verticalWall");
        this.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        verticalWallOpen();
        do {
            timer.reset();
            while (timer.milliseconds() < 800) ;
            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
                timer.reset();
                while (timer.milliseconds() < 500) ;
                target += 120;
            }
        } while (getColor().get(2) == Color.NONE); // 3 датчик нечего не видит

    }


    public void shootingArtefacts() {
        //turn40();
        verticalWallClose();
        horizontalWallOpen();

        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
            while (timer.milliseconds() < 500) {
            }
            target += 120;
        }
        timer.reset();
    }



    public void sortingArtefacts(Scan a, Scan b) { //shooter.b
        //turn40();
        verticalWallOpen();
        while (timer.milliseconds() < 500) ;
        timer.reset();

        this.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        target = 0;

        while (timer.milliseconds() < 2000) ;

        switch (a) {
            case LEFT: {
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
            case RIGHT: {
                switch (b) {
                    case LEFT:
                        target = 280;
                        break;
                    case RIGHT:
                        target = 40;
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
                        target = 160;
                        break;
                    case RIGHT:
                        target = 280;
                        break;
                    case BETWEEN:
                        target = 40;
                        break;
                }
                break;
            }

        }
        timer.reset();
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
        else if (a.get(2) == Color.GREEN) position = Scan.RIGHT;
        else  position = Scan.BETWEEN; //(a.get(0) == Color.GREEN)

        return position;
    }

    public String printGetColor(ArrayList<Color> a){
        String res = "";
        for (int i = 0; i<3; i++){
            res += a.get(i) + " ";
        }
        return res;
    }

    public void drumStop() {
        drumMotor.setPower(0);
    }

    public void horizontalWallClose() {
        horizontalWall.setPosition(HCLOSE_WALL);
    }

    public void verticalWallClose() {
        verticalWall.setPosition(VCLOSE_WALL);
    }

    public void horizontalWallOpen() {
        horizontalWall.setPosition(HOPEN_WALL);
    }

    public void verticalWallOpen() {
        verticalWall.setPosition(VOPEN_WALL);
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



