package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.Angle;

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
    public double R = 0.1;
    public double differenceVelocity = 8;

    public double z = gateZ - highOfShooting;

    public double sv = cMTI(0.017);

    public double target = 0;
    public double angleOfTurret = 0;

    public double angleOfAdjuster;
    double ITM = 0.0254;
    double g = 9.81;
    double maxAngle = 60;
    double minAngle = 45;
    double minServoPos = 0;
    double maxServoPos = 0.5;





    public AutoSniper() {
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

        target = -(angleOfDrivetrain - angleOfTurret) - 180;
        target = tt.angleNormalising(target);
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
        if (angleOfAdjuster >= maxAngle) angleOfAdjuster = maxAngle;
        if (angleOfAdjuster <= minAngle) angleOfAdjuster = minAngle;
        sh.setAngleAdjuster(convertAngleToServoPos(angleOfAdjuster));
    }

    public void continuousSetVelocity(double x, double y, double angleOfDrivetrain, double servoPos, double lastAngularVelocity) {
        double angleOfAdjuster = convertServoPosToAngle(servoPos);
        if (angleOfAdjuster <= minAngle || angleOfAdjuster >= maxAngle) {
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
        return angle * ((maxServoPos-minServoPos) / (maxAngle-minAngle));
    }
    public double convertServoPosToAngle(double servoPos) {
        return servoPos / ((maxServoPos-minServoPos) / (maxAngle-minAngle));
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
