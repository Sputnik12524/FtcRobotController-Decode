package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;

public class Logger {
    String directoryPath = "/sdcard/FIRST/";
    FileWriter writer;

    public Logger(String fileName) {
        try {
            this.writer = new FileWriter(this.directoryPath + fileName + ".csv", false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void addLine (double time, double velocity){
        try {
            writer.write(time + "," + velocity + "\n");
        } catch (IOException e) {
           e.printStackTrace();
        }
    }
    public  void addHeader(String header){
        try {
            writer.write(header + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void fileClose(){
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
