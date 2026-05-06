package org.firstinspires.ftc.teamcode.util;



import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {
    Telemetry telemetry = new MultipleTelemetry();
    ElapsedTime timer = new ElapsedTime();
    private double max_time;
    private double cycles;
    private double all_time;

    public void update(){
        cycles ++;
        all_time += max_time;
        max_time = Math.max(timer.milliseconds(), max_time);
        timer.reset();
    }
    public double getAverage(){
        if(cycles!= 0)return all_time/cycles;
        else return 0;
    }
    public double getMax(){
        return max_time;
    }
    public double getAll(){
        return all_time;
    }
    public double getCycles(){
        return cycles;
    }
    public void updateTelemetry(){
        telemetry.addData("Max cycle",getMax());
        telemetry.addData("Average cycle", getAverage());
        telemetry.addData("Cycles", getCycles());
    }

}
