package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Transfer.Color;

import java.util.ArrayList;

public class Backlight {
    Transfer tr;
    public DigitalChannel P1;
    public DigitalChannel P2;
    public DigitalChannel P3;
    public DigitalChannel P4;
    public DigitalChannel P5;
    public DigitalChannel P6;
    public DigitalChannel G1;
    public DigitalChannel G2;
    public DigitalChannel G3;
    public DigitalChannel G4;
    public DigitalChannel G5;
    public DigitalChannel G6;
    final int  timers = 300;
    ElapsedTime timer;

    public Backlight(LinearOpMode linearOpMode, Transfer transfer) {
        P1 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P1");
        P2 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P2");
        P3 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P3");
        P4 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P4");
        P5 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P5");
        P6 = linearOpMode.hardwareMap.get(DigitalChannel.class, "P6");
        
        G1 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G1");
        G2 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G2");
        G3 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G3");
        G4 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G4");
        G5 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G5");
        G6 = linearOpMode.hardwareMap.get(DigitalChannel.class, "G6");

        this.tr = transfer;
        timer = new ElapsedTime();
        P1.setMode(DigitalChannel.Mode.OUTPUT);
        P2.setMode(DigitalChannel.Mode.OUTPUT);
        P3.setMode(DigitalChannel.Mode.OUTPUT);
        P4.setMode(DigitalChannel.Mode.OUTPUT);
        P5.setMode(DigitalChannel.Mode.OUTPUT);
        P6.setMode(DigitalChannel.Mode.OUTPUT);

        G1.setMode(DigitalChannel.Mode.OUTPUT);
        G2.setMode(DigitalChannel.Mode.OUTPUT);
        G3.setMode(DigitalChannel.Mode.OUTPUT);
        G4.setMode(DigitalChannel.Mode.OUTPUT);
        G5.setMode(DigitalChannel.Mode.OUTPUT);
        G6.setMode(DigitalChannel.Mode.OUTPUT);
        turnOffBacklight();
    }

    public void lights() {
        if (timer.milliseconds() > timers) {
            ArrayList<Color> list = tr.getColor();
            if (list.get(0) == Color.GREEN) {
                P1.setState(false);
                G1.setState(true);
            }
            else if(list.get(0) == Color.PURPLE){
                P1.setState(true);
                G1.setState(false);
            }
            else{
                P1.setState(false);
                G1.setState(false);
            }

            if(list.get(1) == Color.GREEN){
                P2.setState(false);
                G2.setState(true);
            }
            else if (list.get(1) == Color.PURPLE) {
                P2.setState(true);
                G2.setState(false);
            }
            else {
                P2.setState(false);
                P2.setState(false);
            }

            if(list.get(2) == Color.GREEN){
                P3.setState(false);
                G3.setState(true);
            }
            else if(list.get(2) == Color.PURPLE){
                P3.setState(true);
                G3.setState(false);
            }
            else{
                P3.setState(false);
                G3.setState(false);
            }
            timer.reset();
        }
    }
    public void turnOffBacklight() {
        P1.setState(false);
        G1.setState(false);
        P2.setState(false);
        P2.setState(false);
        P3.setState(false);
        G3.setState(false);
    }
}
