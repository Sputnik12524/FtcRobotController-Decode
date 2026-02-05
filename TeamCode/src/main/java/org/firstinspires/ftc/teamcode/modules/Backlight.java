package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Backlight {
    public DigitalChannel green;
    public DigitalChannel purple;
    public Backlight(LinearOpMode linearOpMode){
        green = linearOpMode.hardwareMap.get(DigitalChannel.class, "green");
        purple = linearOpMode.hardwareMap.get(DigitalChannel.class, "purple");

        green.setMode(DigitalChannel.Mode.OUTPUT);
        purple.setMode(DigitalChannel.Mode.OUTPUT);
        turnOffBacklight();
    }
    public void detectedGreen(){
        green.setState(true);
        purple.setState(false);
    }
    public void detectedPurple(){
        purple.setState(true);
        green.setState(false);
    }
    public void glowWhite(){
        green.setState(true);
        purple.setState(true);
    }
    public void turnOffBacklight(){
        green.setState(false);
        purple.setState(false);
    }
}
