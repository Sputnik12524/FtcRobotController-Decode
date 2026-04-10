package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Cycle {
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

}
