package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.Shooter;

public class AutoSniper {

    Turret tt;
    Shooter sh;

    //---------------------------------------------- GENERAL COEFFICIENTS
    public double gateY = 138;
    public double gateX = 138; // Изначально для красного
    public double gateZ = 0;

    public double highOfShooting = 0;
    public double R = 0.1; // meters
    public double differenceVelocity = 8;

    public double z = gateZ - highOfShooting;

   public double sv = 0.017; // meters

    public double target = 0;
    public double angleOfTurret = 0;

    public double angleOfAdjuster;
    double ITM = 0.0254;
    double g = 9.81;
    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = 45;
    public static double POS_FOR_MAX_ANGLE = 0.125;
    public static double POS_FOR_MIN_ANGLE = 0.005;




    public AutoSniper(Turret turret, Shooter shooter) {
        tt = turret;
        sh = shooter;
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                gateX = 6;
                break;
            case RED:
                gateX = 138;
                break;
        }
    }

    public void continuousTurnTurretToGate(double x, double y, double angleOfDrivetrain) {
        if (x <= 0) x = 1;
        if (x >= 144) x = 143;
        double sY = cMTI(sv * Math.sin(angleOfDrivetrain));
        double sX = cMTI(sv * Math.cos(angleOfDrivetrain));

        angleOfTurret = Math.toDegrees(Math.atan( (gateY - (y+sY)) / (gateX - (x+sX)) ));

        target = -(Math.toDegrees(angleOfDrivetrain) - angleOfTurret) - 180;
        target = tt.angleNormalising(tt.stabilizeTargetByCamera(target));
        tt.turnByTarget(target);
    }

    public void continuousSetAngle(double servoPos, double x, double y, double angleOfDrivetrain, double angularVelocity) {
        double lastAngleOfAdjuster = convertServoPosToAngle(servoPos);
        convertServoPosToAngle(servoPos);
        double sY = cMTI(sv * Math.toDegrees(Math.sin(angleOfDrivetrain)));
        double sX = cMTI(sv * Math.toDegrees(Math.cos(angleOfDrivetrain)));
        double l = Math.sqrt( (gateY - (y+sY)) + (gateX - (x+sX)) );
        double a = (g * Math.pow(l,2)) / (2 * Math.pow(cAVTV(angularVelocity),2));
        double c = a - z;
        double D = Math.pow(l,2) - 4*a*c;
        if (D > 0) {
            double var1 = Math.toDegrees(Math.atan( (-l + Math.sqrt(D)) / (2*a) ));
            double var2 = Math.toDegrees(Math.atan( (-l - Math.sqrt(D)) / (2*a) ));
            if (var1 >= 0 && Math.abs(var1-lastAngleOfAdjuster) <= Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjuster = var1;
            } else if (var2 >= 0 && Math.abs(var1-lastAngleOfAdjuster) > Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjuster = var2;
            } else {
                angleOfAdjuster = lastAngleOfAdjuster;
            }
        } else if (D == 0) {
            angleOfAdjuster = Math.toDegrees(Math.atan( (-l) / (2*a) ));
        } else {
            angleOfAdjuster = lastAngleOfAdjuster;
        }
        if (angleOfAdjuster >= MAX_ANGLE) angleOfAdjuster = MAX_ANGLE;
        if (angleOfAdjuster <= MIN_ANGLE) angleOfAdjuster = MIN_ANGLE;
        sh.setAngleAdjuster(convertAngleToServoPos(angleOfAdjuster));
    }

    public void continuousSetVelocity(double x, double y, double angleOfDrivetrain, double servoPos, double lastAngularVelocity) {
        double angleOfAdjuster = convertServoPosToAngle(servoPos);
        if (angleOfAdjuster <= MIN_ANGLE || angleOfAdjuster >= MAX_ANGLE) {
            double sY = cMTI(sv * Math.sin(angleOfDrivetrain));
            double sX = cMTI(sv * Math.cos(angleOfDrivetrain));
            double l = Math.sqrt( (gateY - (y+sY)) + (gateX - (x+sX)) );
            sh.setVelocityTarget(
                    cVTAV( Math.sqrt( (g*Math.pow(l,2) * (1+Math.tan(angleOfAdjuster)))) / ( (Math.tan(angleOfAdjuster - z) * 2)))
                    + differenceVelocity
            );
        } else {
            sh.setVelocityTarget(lastAngularVelocity);
        }
    }



    public double convertAngleToServoPos(double angle) {
        return angle * ( Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE) / Math.abs(MAX_ANGLE - MIN_ANGLE) );
    }
    public double convertServoPosToAngle(double servoPos) {
        return servoPos / ( Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE) / Math.abs(MAX_ANGLE - MIN_ANGLE) );
    }
    public double cITM(double inch) { //Convert inches to meters
        return inch * ITM;
    }
    public double cMTI(double meter) { //Convert meters to inches
        return meter / ITM;
    }
    public double cAVTV(double angularV) { //Convert Angular Velocity to velocity
        return angularV * (2 * Math.PI * R);
    }
    public double cVTAV(double v) { //Convert velocity to Angular Velocity
        return v / (2 * Math.PI * R);
    }


}
