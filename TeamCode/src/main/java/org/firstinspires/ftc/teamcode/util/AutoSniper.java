package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.Shooter;

public class AutoSniper {

    public Turret tt;
    public Shooter sh;

    //---------------------------------------------- GENERAL COEFFICIENTS
    public double goalY = 138;
    public double goalX = 138; // Изначально для красного
    public double goalZ = 51.5;
    public Alliance alliance = Alliance.NONE;

    public double highOfShooting = 9.84;
    public double R = 0.1; // meters
    public double differenceVelocity = 0;
    public boolean AIMING_ACTIVE = true;
    public double targetVeloForArtifact = 0;
    public double targetVelo = 0;

    public double z = goalZ - highOfShooting;

   public double sv = 0.017; // meters

    public double target = 0;
    public double targetBonus = 0;
    public double angleOfTurret = 0;

    public double angleOfAdjuster;
    public double angleOfAdjusterBeforeNormalising;
    public double angleByFormulas;

    double ITM = 0.0254;
    double g = 9.81;

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = 45;
    public static double POS_FOR_MAX_ANGLE = 0.125;
    public static double POS_FOR_MIN_ANGLE = 0.005;

    public double sX, sY, c, a, D, l = 0;
    public double v1, v2, l1, l2 = 0;
    public double minDistanceForLongThrowMode = 3;

    public boolean isCalculateNewVelocity = false;
    public boolean isCalculateNewAngle = false;

    double[] ValuesOfVelocity = {40, 51, 56, 58, 61, 63, 69.45}; // {43, 51, 60, 62-63, 63, 67, 75}
    double[] ValuesOfDistance = {1.0755, 1.825, 2.4381, 2.8065, 3.2954, 3.5351, 3.9292};

    public boolean isShort, isLong = false;




    public AutoSniper(Turret turret, Shooter shooter) {
        tt = turret;
        sh = shooter;
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                goalX = 1;
                this.alliance = Alliance.BLUE;
                break;
            case RED:
                this.alliance = Alliance.RED;
                goalX = 138;
                break;
            case NONE:
                this.alliance = Alliance.NONE;
        }
    }

    public void continuousTurnTurretToGate(double x, double y, double angleOfDrivetrain) {
        if (AIMING_ACTIVE) {
            if (x <= 0) x = 1;
            if (x >= 144) x = 143;

        switch (alliance) {
            case RED:
                angleOfTurret = Math.toDegrees(Math.atan( (goalY - (y+sY)) / (goalX - (x+sX)) ));
                target = -(Math.toDegrees(angleOfDrivetrain) - angleOfTurret); // - 180;
                break;
            case BLUE:
                angleOfTurret = 180 - Math.toDegrees(Math.atan( (goalY - (y+sY)) / ((x+sX) - goalX) ));
                target = -(Math.toDegrees(angleOfDrivetrain) - angleOfTurret); // - 180;
                break;
            case NONE:
                angleOfTurret = 0;
        }

           // target = tt.angleNormalising(tt.stabilizeTargetByCamera(target) + targetBonus);
            target = tt.angleNormalising(target);
            tt.turnByTarget(target);
        } else {
            tt.turnByTarget(0);
        }
    }

    public void continuousSetAngleByFormula(double servoPos) {
        double lastAngleOfAdjuster = convertServoPosToAngle(servoPos);
        if (D > 0) {
            isCalculateNewAngle = true;
            double var1 = Math.toDegrees(Math.atan( (-l + Math.sqrt(D)) / (2*a) ));
            double var2 = Math.toDegrees(Math.atan( (-l - Math.sqrt(D)) / (2*a) ));
            if (var1 >= 0 && Math.abs(var1-lastAngleOfAdjuster) <= Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjusterBeforeNormalising = var1;
                angleByFormulas = angleOfAdjusterBeforeNormalising;
            } else if (var2 >= 0 && Math.abs(var1-lastAngleOfAdjuster) > Math.abs(var2-lastAngleOfAdjuster)) {
                angleOfAdjusterBeforeNormalising = var2;
                angleByFormulas = angleOfAdjusterBeforeNormalising;
            } else {
                isCalculateNewAngle = false;
                angleOfAdjusterBeforeNormalising = lastAngleOfAdjuster;
                angleByFormulas = -1;
            }
        } else if (D == 0) {
            isCalculateNewAngle = true;
            angleOfAdjusterBeforeNormalising = Math.toDegrees(Math.atan( (-l) / (2*a) ));
            angleByFormulas = angleOfAdjusterBeforeNormalising;
        } else {
            isCalculateNewAngle = false;
            angleOfAdjusterBeforeNormalising = lastAngleOfAdjuster;
            angleByFormulas = -1;
        }
        angleOfAdjuster = angleOfAdjusterBeforeNormalising;
        if (angleOfAdjusterBeforeNormalising >= MAX_ANGLE) angleOfAdjuster = MAX_ANGLE;
        if (angleOfAdjusterBeforeNormalising <= MIN_ANGLE) angleOfAdjuster = MIN_ANGLE;
        sh.setAngleAdjuster(convertAngleToServoPos(angleOfAdjuster));
    }

    public void setAngleByLocalisation(double l, double servoPos) {
        if (l <= minDistanceForLongThrowMode && servoPos != Shooter.VELOCITY_FOR_SHORT_THROW) {
            sh.setShortThrowMode();
            isLong = false;
            isShort = true;
        } else if (l > minDistanceForLongThrowMode && servoPos != Shooter.VELOCITY_FOR_LONG_THROW) {
            sh.setLongThrowMode();
            isLong = true;
            isShort = false;
        }
    }

    public void continuousSetVelocityTargetByFormula(double servoPos, double lastAngularVelocity) {
        double angleOfAdjuster = Math.toRadians( convertServoPosToAngle(servoPos) );
        if ( (angleOfAdjusterBeforeNormalising <= MIN_ANGLE || angleOfAdjusterBeforeNormalising >= MAX_ANGLE)
        && (angleOfAdjusterBeforeNormalising != 0) ) {
            isCalculateNewVelocity = true;
            targetVelo = cVTAV(Math.sqrt( (g*Math.pow(l,2) * (1+Math.tan(angleOfAdjuster)))) / ( (Math.tan(angleOfAdjuster) * l - cITM(z)) * 2 ));
            sh.setVelocityTarget(targetVelo + differenceVelocity);
        } else {
            isCalculateNewVelocity = false;
            sh.setVelocityTarget(lastAngularVelocity);
        }
    }
    public void continuousSetVelocityTargetByInterpol() {
        for (int i=0;  i<ValuesOfDistance.length-1;  i++) {
            if (l > ValuesOfDistance[5]) {
                l = ValuesOfDistance[5];
            } else if (l < ValuesOfDistance[0]){
                l = ValuesOfDistance[0];
            }
            if (ValuesOfDistance[i] <= l && l <= ValuesOfDistance[i+1]) {
                v1 = ValuesOfVelocity[i];
                v2 = ValuesOfVelocity[i];
                l1 = ValuesOfDistance[i];
                l2 = ValuesOfDistance[i+1];

                targetVelo = v1 + ( ((l-l1)/(l2-l1))*(v2-v1) );
                sh.setVelocityTarget(targetVelo);
            }
        }
    }



    public void continuousCalculateGeneralValues(double x, double y, double angleOfDrivetrain, double angularVelocity) {
        sY = cMTI(sv * Math.sin(angleOfDrivetrain)); //inches
        sX = cMTI(sv * Math.cos(angleOfDrivetrain)); //inches
        l = cITM(Math.sqrt(Math.pow((goalY - (y + sY)), 2) + Math.pow((goalX - (x + sX)), 2))); //meters
        a = g * Math.pow(l, 2) / (2 * Math.pow(angularVelocity, 2));
        c = a - cITM(z);
        D = Math.pow(l, 2) - 4 * a * c;
    }


    public double convertAngleToServoPos(double angle) {
        return POS_FOR_MIN_ANGLE + ((angle - MIN_ANGLE) * ( Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE) / Math.abs(MAX_ANGLE - MIN_ANGLE) ));
    }
    public double convertServoPosToAngle(double servoPos) {
        return MIN_ANGLE + ((servoPos - POS_FOR_MIN_ANGLE) * ( Math.abs(MAX_ANGLE - MIN_ANGLE) / Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE) ));
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
    public void enableAutoTurretAiming(boolean isEnabled) {
        AIMING_ACTIVE = isEnabled;
    }

}
