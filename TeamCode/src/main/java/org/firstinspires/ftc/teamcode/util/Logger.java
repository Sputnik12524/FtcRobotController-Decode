package org.firstinspires.ftc.teamcode.util;


import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class Logger {
    String directoryPath = "/sdcard/FIRST/";
    BufferedReader reader;
    BufferedWriter writer;
    public String fileName;
    public String line;
    public Alliance al;
    public double x;
    public double y;
    public double degrees;

    public Logger(String fileName) {
        try {
            this.fileName = fileName;
            this.writer = new BufferedWriter(new FileWriter(this.directoryPath + fileName + ".csv", false));
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

    public void writePose(Alliance alliance, double x, double y, double degrees) {
        try {
            switch (alliance) {
                case RED:
                    writer.write("Red" + "," + x + "," + y + "," + degrees);
                    break;
                case BLUE:
                    writer.write("Blue" + "," + x + "," + y + "," + degrees);
                    break;
                default:
                    writer.write("none");
            }
        } catch (IOException exe) {
            exe.printStackTrace();
        }
    }

    public void getAll() {
        try {
            this.reader = new BufferedReader(new FileReader(this.directoryPath + fileName + ".csv"));
            line = reader.readLine();
            if (line != null) {
                String[] a = line.split(",");
                if (a[0].equals("Blue")) al = Alliance.BLUE;
                else if (a[0].equals("Red")) al = Alliance.RED;
                else al = Alliance.NONE;
                if (a[1] != null) x = Double.parseDouble(a[1]);
                if (a[2] != null) y = Double.parseDouble(a[2]);
                if(a[3] != null) degrees = Double.parseDouble(a[3]);
            }
            reader.close();
        } catch (IOException exe) {
            exe.printStackTrace();
        }
    }

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
