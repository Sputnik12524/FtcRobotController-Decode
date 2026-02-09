package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.modules.Transfer.Color;

import java.util.ArrayList;

public class Backlight {
    Transfer tr;
    public DigitalChannel D1;
    public DigitalChannel D2;
    public DigitalChannel D3;
    public DigitalChannel D4;
    public DigitalChannel D5;
    public DigitalChannel D6;
    public Backlight(LinearOpMode linearOpMode, Transfer transfer){
        D1 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D1");
        D2 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D2");
        D3 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D3");
        D4 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D4");
        D5 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D5");
        D6 = linearOpMode.hardwareMap.get(DigitalChannel.class, "D6");
        this.tr  = transfer;
        D1.setMode(DigitalChannel.Mode.OUTPUT);
        D2.setMode(DigitalChannel.Mode.OUTPUT);
        D3.setMode(DigitalChannel.Mode.OUTPUT);
        D4.setMode(DigitalChannel.Mode.OUTPUT);
        D5.setMode(DigitalChannel.Mode.OUTPUT);
        D6.setMode(DigitalChannel.Mode.OUTPUT);
        turnOffBacklight();
    }
    public void lights(){
        ArrayList<Color> list = tr.getColor();
//        if(list.get(0) == Color.GREEN) D1.setState();
            
    }
    public void detectedGreen(){
        D1.setState(true);
        D2.setState(false);
    }
    public void detectedPurple(){
        D2.setState(true);
        D1.setState(false);
    }
    public void glowWhite(){
        D1.setState(true);
        D2.setState(true);
    }
    public void turnOffBacklight(){
        D1.setState(false);
        D2.setState(false);
    }
}
