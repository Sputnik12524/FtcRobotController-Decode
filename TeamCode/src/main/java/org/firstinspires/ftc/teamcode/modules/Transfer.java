package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;

public class Transfer {
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;

    public enum Color {GREEN, PURPLE, NONE}
    public enum Num{FIRST, SECOND, THIRD}
    enum Comb {GGG, GGP, GPP, PGG, PPG, PPP, NONE}
    Comb comb = Comb.NONE;

    public final float[] hsv1 = new float[3];
    public final float[] hsv2 = new float[3];
    private final float[] hsv3 = new float[3];


    //---------------------------------------------- DASHBOARD
    public static double GREEN_MAX = 175;// 0.65   0.9
    public static double GREEN_MIN = 140;//0.3   0.5
    public static double PURPLE_MAX = 250;
    public static double PURPLE_MIN = 180;
    public final float GAIN = 3.4f;

    public Transfer(LinearOpMode opMode) {
        colorSensor1 = opMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor1.setGain(GAIN);
        colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor2.setGain(GAIN);
        colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");
        colorSensor3.setGain(GAIN);
    }

    public ArrayList<Transfer.Color> getColor() {
        ArrayList<Transfer.Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
        float r1 = color1.red;
        float g1 = color1.green;
        float b1 = color1.blue;
        android.graphics.Color.RGBToHSV(
                (int)(r1 * 255),
                (int)(g1 * 255),
                (int)(b1 * 255),
                hsv1
        );
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        float r2 = color2.red;
        float g2 = color2.green;
        float b2 = color2.blue;
        android.graphics.Color.RGBToHSV(
                (int)(r2 * 255),
                (int)(g2 * 255),
                (int)(b2 * 255),
                hsv2        );
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
        float r3 = color3.red;
        float g3 = color3.green;
        float b3 = color3.blue;
        android.graphics.Color.RGBToHSV(
                (int)(r3* 255),
                (int)(g3 * 255),
                (int)(b3 * 255),
                hsv3
        );


        if ((hsv1[0] <= GREEN_MAX && hsv1[0] >= GREEN_MIN ) )  {
            colorSensors.add(Transfer.Color.GREEN);
        } else if ((hsv1[0] <= PURPLE_MAX && hsv1[0] >= PURPLE_MIN )) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);


        if ((hsv2[0] <= GREEN_MAX && hsv2[0] >= GREEN_MIN )) {
            colorSensors.add(Transfer.Color.GREEN);
        } else if ((hsv2[0] <= PURPLE_MAX && hsv2[0] >= PURPLE_MIN )) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);


        if ((hsv3[0] <= GREEN_MAX && hsv3[0] >= GREEN_MIN ) )  {
            colorSensors.add(Transfer.Color.GREEN);
        } else if ((hsv3[0] <= PURPLE_MAX && hsv3[0] >= PURPLE_MIN )) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);

        return colorSensors;
    }

    public int howMany(){
        int art = 0;
        ArrayList<Color> list = getColor();
        for(Color c: list){
            if(c != Color.NONE) art++;
        }
        return art;
    }

//    public Comb scanArt(){
//        ArrayList<Color> list = getColor();
//        if(list.get(0) == Color.GREEN && list.get(1) == Color.GREEN && list.get(2) == Color.GREEN)
//
//    }

    public boolean isEmpty() {
       ArrayList<Color> list = getColor();
       for(Color col: list){
           if(col != Color.NONE) return false;
       }
        return true;
    }
}
