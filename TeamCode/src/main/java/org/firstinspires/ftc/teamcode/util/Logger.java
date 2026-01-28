package org.firstinspires.ftc.teamcode.util;


import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class Logger {
    String directoryPath = "/sdcard/FIRST/";
    FileWriter writer;
    BufferedReader reader;
    public Logger(String fileName) {
        try {
            this.writer = new FileWriter(this.directoryPath + fileName + ".csv", false);
            this.reader = new BufferedReader(new FileReader(this.directoryPath + fileName + ".csv"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addLine(double time, double velocity) {
        try {
            writer.write(time + "," + velocity + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writePose(Alliance alliance, double x, double y) {
        try {
            switch (alliance){
                case NONE:
                    writer.write("none");
                case RED:
                    writer.write("Red" + "," + x + "," + y + "\n");
                case BLUE:
                    writer.write("blue" + "," + x + "," + y + "\n");
                default:
                    writer.write("none");
            }

        } catch (IOException exe) {
            exe.printStackTrace();
        }
    }

//    public String getAlliance(){
//        try{
//            return reader.
//        }catch (IOException exe){
//            exe.printStackTrace();
//        }
//    }


    public void addHeader(String header) {
        try {
            writer.write(header + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void fileClose() {
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
