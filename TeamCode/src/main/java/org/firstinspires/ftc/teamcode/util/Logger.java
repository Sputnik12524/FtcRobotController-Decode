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
    BufferedWriter ampsWriter;
    public String fileName;
    public String line;
    public Alliance al;
    public double x;
    public double y;
    public double heading;
    public double turretPose;
    public double oldTurretPose;

    public Logger(String fileName) {
        this.fileName = fileName;
    }

    public void addLine(double time, double velocity) {
        try {
            writer.write(time + "," + velocity + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writePose(Alliance alliance, double x, double y, double heading, double turretPose) {
        try {
            this.writer = new BufferedWriter(new FileWriter(this.directoryPath + fileName + ".csv", false));

            switch (alliance) {
                case RED:
                    writer.write("Red" + "," + x + "," + y + "," + heading + "," + oldTurretPose);
                    break;
                case BLUE:
                    writer.write("Blue" + "," + x + "," + y + "," + heading + "," + oldTurretPose);
                    break;
                default:
                    writer.write("none");
            }
            writer.flush();
            if (turretPose != 0) {
                oldTurretPose = turretPose;
            }
        } catch (IOException exe) {
            exe.printStackTrace();
        }
    }


    public void getAll(String fileName) throws IOException {

        this.reader = new BufferedReader(new FileReader(this.directoryPath + fileName + ".csv"));
        line = reader.readLine();
        if (line != null) {
            String[] a = line.split(",");
            if (a[0].equals("Blue")) al = Alliance.BLUE;
            else if (a[0].equals("Red")) al = Alliance.RED;
            else al = Alliance.NONE;
            if (a[1] != null) x = Double.parseDouble(a[1]);
            if (a[2] != null) y = Double.parseDouble(a[2]);
            if (a[3] != null) heading = Double.parseDouble(a[3]);
            if (a[4] != null) turretPose = Double.parseDouble(a[4]);
        }
        reader.close();
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


