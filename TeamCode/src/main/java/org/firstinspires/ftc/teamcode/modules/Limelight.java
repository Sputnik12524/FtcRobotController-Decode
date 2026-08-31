package org.firstinspires.ftc.teamcode.modules;

import static java.lang.Math.abs;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public class Limelight {
    public final Limelight3A limelight3A;
    Turret tt;
    Follower fl;
    final LinearOpMode opMode;
    public LLResult cachedResult;

    public final double X_RESOLUTION = 1280;
    public double X_MIDDLE = X_RESOLUTION / 2;
    public final double Y_RESOLUTION = 960;
    public double Y_MIDDLE = Y_RESOLUTION / 2;
    final double turret_offset_x = -3.073 / 1000;
    final double turret_offset_y = 5.242 / 1000;
    final double camera_offset_x = 155.057 / 1000;
    final double camera_offset_y = 0;
    public final LimelightThread lt = new LimelightThread();
    final ElapsedTime timer = new ElapsedTime();


    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        timer.reset();
    }

    public Limelight(LinearOpMode opMode, Turret tt, Follower fl) {
        this.opMode = opMode;
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        this.tt = tt;
        this.fl = fl;
        limelight3A.pipelineSwitch(0);
        timer.reset();
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
    public double[] getTagInfo() {
        update();
        double[] tagInfo = new double[4];
        LLResult result = getResult();
        double id = 0;
        if (!result.getFiducialResults().isEmpty()) {
            id = result.getFiducialResults().get(0).getFiducialId();
        }

        double tx = result.getTx();
        double poseX = result.getBotpose().getPosition().x;
        double poseY = result.getBotpose().getPosition().y;
        tagInfo[0] = id;
        tagInfo[1] = tx;
        tagInfo[2] = poseX;
        tagInfo[3] = poseY;

        return tagInfo;
    }

    public double[] getGoalTag() {
        double[] tag = getTagInfo();
        if (tag[0] == 24 || tag[0] == 20) {
            return tag;
        } else {
            if (cachedResult == null || !cachedResult.isValid()) {
                return new double[]{0, 0, 0, 0};
            }
            return getTagInfo();
        }
    }

    public class LimelightThread extends Thread {
        @Override
        public void run() {
            while (!isInterrupted()) {
                getGoalTag();
            }
        }
    }

    //------------------------------------------------КРИНЖ ПОЗА ПО APRIL TAG
    public Pose getPoseByAprilTag() {
        Pose pp;
        LLResult result = limelight3A.getLatestResult();
        Pose3D botpose = result.getBotpose();
        try {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            double head = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            x = 72 - x * 39.37;
            y = 72 + y * 39.37;
//            double[] pose = calculatePose(x, y, head);
            pp = new Pose(y, x, head, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);//помеял все окей
            return pp;
        } catch (Exception e) {
            return new Pose(11, 11, 11);
        }
    }

    public double[] getRawPose() {
        LLResult res = getResult();
        if (!res.isValid()) return new double[]{0, 0, 0};
        else {
            return new double[]{
                    res.getBotpose().getPosition().x,
                    res.getBotpose().getPosition().y,
                    res.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES)
            };
        }
    }

    public void relocalizeWhenError() {
        Pose tagPose = getPoseByAprilTag();
        if (tagPose == null) return;
        double[] poseOdo = {fl.getPose().getX(), fl.getPose().getY(), fl.getHeading()};
        double[] poseTag = {getPoseByAprilTag().getX(), getPoseByAprilTag().getY(), getPoseByAprilTag().getHeading()};
        double[] localizationErrors = comparePose(poseOdo, poseTag);


        if (abs(localizationErrors[0]) > 2 || abs(localizationErrors[1]) > 2
                || abs(localizationErrors[2]) > 5) {
            fl.setPose(tagPose);
        }
    }

    public void relocalizeInNSeconds(double n) {
        if (timer.milliseconds() >= n) {
            relocalizeWhenError();
            timer.reset();
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

        double[] c_in_t = {rotateVector(camera_offset_x, camera_offset_y, tt.getCurrentPosOfTurret())[0],
                rotateVector(camera_offset_x, camera_offset_y, tt.getCurrentPosOfTurret())[1]};
        double[] t_in_r = {turret_offset_x, turret_offset_y};
        double[] c_in_r = {
                t_in_r[0] + c_in_t[0],
                t_in_r[1] + c_in_t[1]
        };
        return new double[]{
                x + c_in_r[0],
                y + c_in_r[1],
                heading - tt.getCurrentPosOfTurret()
        };
    }

    public double[] rotateVector(double x, double y, double theta) {
        double[] rotatedVector = new double[2];
        theta = Math.toRadians(theta);
        rotatedVector[0] = (x * Math.cos(theta) - y * Math.sin(theta));
        rotatedVector[1] = (x * Math.sin(theta) + y * Math.cos(theta));
        return rotatedVector;
    }

    //-----------------------------------------------------------GETTING ARTIFACTS FROM THE NEURAL DETECTOR
// WARNING: (SWITCH PIPELINE IN WEB INTERFACE BEFORE USING!)
    public String getArtifactClassname() {
        LLResult result = getResult();
        String classname = " ";
        List<LLResultTypes.DetectorResult> detectorRes = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorRes) {
            classname = dr.getClassName();
        }
        return classname;
    }

    //------------------------------------------------------GETTING STATUS & RESULT FOR CAMERA WORK

    public void update() {
        cachedResult = limelight3A.getLatestResult();
    }

    public LLResult getResult() {
        return cachedResult;
    }

    public LLStatus limelightStatus() {
        return limelight3A.getStatus();
    }


}
