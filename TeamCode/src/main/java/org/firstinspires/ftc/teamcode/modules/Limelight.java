package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;
@Config
public class Limelight {
    public Limelight3A limelight3A;
    LinearOpMode opMode;
    public double X_RESOLUTION = 1280;
    public double X_MIDDLE = X_RESOLUTION/2;
    public double Y_RESOLUTION = 960;
    public double Y_MIDDLE = Y_RESOLUTION/2;

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }

    public void startOrStopLL(boolean isStarted) {
        if (isStarted) {
            limelight3A.stop();
        } else {
            limelight3A.start();
        }
    }

    public ArrayList<Double> getTagInfo() {
        ArrayList<Double> tagInfo = new ArrayList<>();
        LLResult result = limelightResult();
        double id = 0;
        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fr : fidResults) {
                id = fr.getFiducialId();
            }
        } else {
            opMode.telemetry.addData("Error, no data available", limelightStatus());
        }
        double tx = result.getTx();
        double ty = result.getTy();
        tagInfo.add(id);
        tagInfo.add(tx);
        tagInfo.add(ty);
        return tagInfo;
    }

    //---------------------------------------------- GETTING
    public LLResult limelightResult() {
        return limelight3A.getLatestResult();
    }
    public LLStatus limelightStatus() {
        return limelight3A.getStatus();
    }
    public Position getPoseByAprilTag(){
        Position pose;
        if(limelightResult().isValid()) pose = limelightResult().getBotpose().getPosition();
        else pose = null;
        return pose;
    }
}
