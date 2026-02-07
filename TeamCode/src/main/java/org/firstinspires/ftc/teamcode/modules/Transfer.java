package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;

public class Transfer {
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;

    enum Color {GREEN, PURPLE, NONE}

    enum Pos {GGG, GGP, GPP, PGG, PPG, PPP, NONE}

    private final float[] hsv1 = new float[3];
    private final float[] hsv2 = new float[3];
    private final float[] hsv3 = new float[3];


    //---------------------------------------------- DASHBOARD
    public static double GREEN_MAX = 175;
    public static double GREEN_MIN = 115;
    public static double PURPLE_MAX = 245;
    public static double PURPLE_MIN = 180;
    public final float GAIN = 2.4f;

    public Transfer(LinearOpMode opMode) {
        colorSensor1 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor1.setGain(GAIN);
        colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor2.setGain(GAIN);
        colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");
        colorSensor3.setGain(GAIN);
    }

    public ArrayList<Transfer.Color> getColor() {
        ArrayList<Transfer.Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
        android.graphics.Color.colorToHSV(color1.toColor(), hsv1);
        android.graphics.Color.colorToHSV(color2.toColor(), hsv2);
        android.graphics.Color.colorToHSV(color3.toColor(), hsv3);


        if (hsv1[0] <= GREEN_MAX && hsv1[0] >= GREEN_MIN) {
            colorSensors.add(Transfer.Color.GREEN);
        } else if (hsv1[0] <= PURPLE_MAX && hsv1[0] >= PURPLE_MIN) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);


        if (hsv2[0] <= GREEN_MAX && hsv2[0] >= GREEN_MIN) {
            colorSensors.add(Transfer.Color.GREEN);
        } else if (hsv2[0] <= PURPLE_MAX && hsv2[0] >= PURPLE_MIN) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);


        if (hsv3[0] <= GREEN_MAX && hsv3[0] >= GREEN_MIN) {
            colorSensors.add(Transfer.Color.GREEN);
        } else if (hsv3[0] <= PURPLE_MAX && hsv3[0] >= PURPLE_MIN) {
            colorSensors.add(Transfer.Color.PURPLE);
        } else colorSensors.add(Transfer.Color.NONE);

        return colorSensors;
    }

    public Color get0(ArrayList a) {
        return (Color) a.get(0);
    }
    public Color get1(ArrayList a) {
        return (Color) a.get(1);
    }
    public Color get2(ArrayList a) {
        return (Color) a.get(2);
    }

    public boolean isEmpty() {
        for (int i = 0; i < 3; ++i) {
            if (getColor().get(i) != Transfer.Color.NONE) return false;
        }
        return true;
    }
}
