package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.Shooter;

public class AutoSniper {
    Shooter sh;
    Turret tt;
    Follower follower;

    //---------------------------------------------- GENERAL COEFFICIENTS
    public double gateY = 144;
    public double gateX;
    public double gateZ = 0;

    public double highOfShooting = 0;
    public double differenceVelocity = 8;

    public double z = gateZ - highOfShooting;

    public double sv = 5; //Not changed

    public double target = 0;
    public double angleOfTurret = 0;

    public double angleOfAdjuster;
    double ITM = 0.0254;
    double g = 9.81;




    public AutoSniper() {
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                gateX = 0;
                break;
            case RED:
                gateX = 144;
                break;
        }
    }

    public void continuousTurnTurretToGate(double x, double y, double angleOfDrivetrain) {
        if (x <= 0) x = 1;
        if (x >= 144) x = 143;
        double sY = cMTI(sv * Math.toDegrees(Math.sin(angleOfDrivetrain)));
        double sX = cMTI(sv * Math.toDegrees(Math.cos(angleOfDrivetrain)));

        angleOfTurret = Math.toDegrees(Math.atan( (gateY - (y+sY)) / (gateX - (x+sX)) ));

        target = -(angleOfDrivetrain - angleOfTurret) - 180;
        tt.angleNormalising(target);
        tt.turnByTarget(target);
    }

    public double continuousSetAngle(double servoPos, double x, double y, double angleOfDrivetrain,
                                     double lastAngleOfAdjuster, double angularVelocity) {
        convertServoPosToAngle(servoPos);
        double sY = cMTI(sv * Math.toDegrees(Math.sin(angleOfDrivetrain)));
        double sX = cMTI(sv * Math.toDegrees(Math.cos(angleOfDrivetrain)));
        double l = Math.sqrt( (gateY - (y+sY)) + (gateX - (x+sX)) );
        double a = (g * Math.pow(l,2)) / (2 * Math.pow(cAVTV(angularVelocity),2));
        double c = a - z;
        if (Math.pow(l,2) - 4*a*c > 0) {
            double var1 = Math.toDegrees(Math.atan( (-l + Math.sqrt(Math.pow(l,2) - 4*a*c)) / (2*a) ));
            double var2 = Math.toDegrees(Math.atan( (-l - Math.sqrt(Math.pow(l,2) - 4*a*c)) / (2*a) ));
            if (var1 >= 0 && Math.abs(var1-lastAngleOfAdjuster) <= Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjuster = var1;
            } else if (var2 >= 0 && Math.abs(var1-lastAngleOfAdjuster) > Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjuster = var2;
            } else {
                angleOfAdjuster = lastAngleOfAdjuster;
            }
        } else if (Math.pow(l,2) - 4*a*c == 0) {
            angleOfAdjuster = Math.toDegrees(Math.atan( (-l) / (2*a) ));
        } else {
            angleOfAdjuster = lastAngleOfAdjuster;
        }
        if (angleOfAdjuster >= 60) angleOfAdjuster = 60;
        if (angleOfAdjuster <= 45) angleOfAdjuster = 45;
        return convertAngleToServoPos(angleOfAdjuster);
    }

    public double continuousSetVelocity(double x, double y, double angleOfDrivetrain, double angleOfAdjuster, double lastAngularVelocity) {
        if (angleOfAdjuster >= 45 && angleOfAdjuster <= 60) {
            double sY = cMTI(sv * Math.toDegrees(Math.sin(angleOfDrivetrain)));
            double sX = cMTI(sv * Math.toDegrees(Math.cos(angleOfDrivetrain)));
            double l = Math.sqrt( (gateY - (y+sY)) + (gateX - (x+sX)) );
            return cVTAV(Math.sqrt( (g * Math.pow(l,2) * (1+Math.toDegrees(Math.tan(angleOfAdjuster))))
                    / ((Math.toDegrees(Math.tan(angleOfAdjuster)) - z) * 2) ));
        } else {
            return lastAngularVelocity;
        }
    }



    public double convertAngleToServoPos(double angle) {
        return angle;
    }
    public double convertServoPosToAngle(double servoPos) {
        return servoPos;
    }
    public double cITM(double inch) { //Convert inches to meters
        return inch * ITM;
    }
    public double cMTI(double meter) { //Convert meters to inches
        return meter / ITM;
    }
    public double cAVTV(double angularV) {
        return angularV-8;
    }
    public double cVTAV(double v) {
        return (v) + 8;
    }


}
