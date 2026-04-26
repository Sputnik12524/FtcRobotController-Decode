package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

@Config
public class Limelight {
    public final Limelight3A limelight3A;
    Turret tt;
    Follower fl;
    LinearOpMode opMode;
    public double X_RESOLUTION = 1280;
    public double X_MIDDLE = X_RESOLUTION / 2;
    public double Y_RESOLUTION = 960;
    public double Y_MIDDLE = Y_RESOLUTION / 2;
    ArrayList<Double> tagInfo;

    double turret_offset_x = 1;
    double turret_offset_y = 0;
    double camera_offset_x = 1;
    double camera_offset_y = 1;


    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        tagInfo = new ArrayList<>();

    }

    public Limelight(LinearOpMode opMode, Turret tt, Follower fl) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        this.tt = tt;
        this.fl = fl;
        limelight3A.pipelineSwitch(0);
        tagInfo = new ArrayList<>();

    }

    //--------------------------------------------------------------START OR STOP CAMERA
    public void startOrStopLL(boolean isStarted) {
        if (isStarted) {
            limelight3A.stop();
        } else {
            limelight3A.start();
        }
    }

    //--------------------------------------------------------------GETTING INFO FROM APRIL TAG
    public ArrayList<Double> getTagInfo() {
        tagInfo.clear();
        LLResult result = limelightResult();
        double id = 0;
        List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fidResults) {
            id = fr.getFiducialId();
        }

        double tx = result.getTx();
        double poseX = result.getBotpose().getPosition().x;
        double poseY = result.getBotpose().getPosition().y;
        tagInfo.add(id);
        tagInfo.add(tx);
        tagInfo.add(poseX);
        tagInfo.add(poseY);

        return tagInfo;
    }

    public ArrayList<Double> getGoalTag() {
        if (getTagInfo().get(0) == 24 || getTagInfo().get(0) == 20) {
            return getTagInfo();
        } else {
            tagInfo.clear();
            for (int i = 0; i < 4; i++) {
                tagInfo.add(.0);
            }
            return tagInfo;
        }
    }

    //------------------------------------------------КРИНЖ ПОЗА ПО APRIL TAG
    public Pose getPoseByAprilTag() {
        Pose pp;
        LLResult result = limelightResult();
        if (!result.isValid()) pp = null;
        else {
            double x = result.getBotpose().getPosition().x;
            double y = result.getBotpose().getPosition().y;
            double head = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            double[] pose = calculatePose(x, y, head);
            pp = new Pose(pose[0], pose[1], pose[2], FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
        return pp;
    }

    public void relocalizeWhenError() {
        Pose tagPose = getPoseByAprilTag();
        if (tagPose == null) return;
        double [] poseOdo = {fl.getPose().getX(), fl.getPose().getY(), fl.getHeading()};
        double [] poseTag = {getPoseByAprilTag().getX(), getPoseByAprilTag().getY(), getPoseByAprilTag().getHeading()};
        double [] localizationErrors = comparePose(poseOdo,poseTag);


        if (Math.abs(localizationErrors[0]) > 2 || Math.abs(localizationErrors[1]) > 2
                        || Math.abs(localizationErrors[2]) > 5) {
            fl.setPose(tagPose);
        }
    }

    public double[] comparePose(double[] poseOdo, double[] poseTag) {
        return new double[]{
                poseOdo[0] - poseTag[0],
                poseOdo[1] - poseTag[1],
                poseOdo[2] - poseTag[2]
        };
    }

    public double[] calculatePose(double x, double y, double heading) {

        double[] c_in_t = {rotateVector(camera_offset_x, camera_offset_y, tt.getCurrentPosOfTurret()).get(0),
                rotateVector(camera_offset_x, camera_offset_y, tt.getCurrentPosOfTurret()).get(1)};
        double[] t_in_r = {turret_offset_x, turret_offset_y};
        double[] c_in_r = {
                t_in_r[0] + c_in_t[0],
                t_in_r[1] + c_in_t[1]
        };
        double[] tagPose = {
                x + c_in_r[0],
                y + c_in_r[1],
                heading - tt.getCurrentPosOfTurret()
        };
        return tagPose;
    }

    public ArrayList<Double> rotateVector(double x, double y, double theta) {
        ArrayList<Double> rotatedVector = new ArrayList<>();
        rotatedVector.add(x * Math.cos(theta) - y * Math.sin(theta));
        rotatedVector.add(x * Math.sin(theta) + y * Math.cos(theta));
        return rotatedVector;
    }

    //-----------------------------------------------------------GETTING ARTIFACTS FROM THE NEURAL DETECTOR
// WARNING: (SWITCH PIPELINE IN WEB INTERFACE BEFORE USING!)
    public String getArtifactClassname() {
        LLResult result = limelightResult();
        String classname = " ";
        List<LLResultTypes.DetectorResult> detectorRes = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorRes) {
            classname = dr.getClassName();
        }
        return classname;
    }

    //------------------------------------------------------GETTING STATUS & RESULT FOR CAMERA WORK
    public LLResult limelightResult() {
        return limelight3A.getLatestResult();
    }

    public LLStatus limelightStatus() {
        return limelight3A.getStatus();
    }


}
