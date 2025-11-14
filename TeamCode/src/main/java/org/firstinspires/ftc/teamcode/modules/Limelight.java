package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public class Limelight {
    public Limelight3A limelight3A;
    LinearOpMode opMode;

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }

    public void startOrStopLL(boolean isStarted) {
        if (isStarted){
            limelight3A.stop();
        } else {
            limelight3A.start();
        }
    }

    public LLResult limelightResult() {
        return limelight3A.getLatestResult();
    }
    public LLStatus limelightStatus() {
        return limelight3A.getStatus();
    }

    public int getTagID() {
        LLResult result = limelightResult();
        int id = 0;
        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fr : fidResults) {
                id = fr.getFiducialId();
            }
        } else {
            opMode.telemetry.addData("Error, no data available", limelightStatus());
        }
        return id;
    }
    //либо
    public int getSingleTagID(){
        LLResultTypes.FiducialResult fr = limelightResult().getFiducialResults().get(0);
        return fr.getFiducialId(); //хз пусть так будет надо протестить
    }
}
