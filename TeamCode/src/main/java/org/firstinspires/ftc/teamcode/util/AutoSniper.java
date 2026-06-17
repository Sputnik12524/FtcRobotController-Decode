package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

public class AutoSniper {

    public Turret tt;
    public Shooter sh;
    public Limelight ll;
    Follower follower;


    AimingMethod state = AimingMethod.LOCALIZATION;

    ElapsedTime isSpinUpTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    //---------------------------------------------- GENERAL COEFFICIENTS
    public double goalY = 138;
    public double goalX = 138; // Изначально для красного
    public double goalZ = 51.5;
    public Alliance alliance = Alliance.NONE;

    public double interpolPose = 70;

    public double highOfShooting = 9.84;
    public double R = 0.1; // meters
    public double differenceVelocity = 0;
    public boolean AIMING_ACTIVE = true;
    public double targetVeloForArtifact = 0;
    public double targetVelo = 0;
    public double mainVelo = 52;
    public double targetAngle = 0;

    public double z = goalZ - highOfShooting;

    public double sv = 0.017; // meters

    public double target = 0;
    public double targetBonus = 0;
    public double driverTargetBonus = 0;
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

    public static double IS_SPIN_UP = 1;

    public double sX, sY, c, a, D, l = 0;
    public double v1, v2, l1, l2 = 0;
    public double minDistanceForLongThrowMode = 2.63;

    public boolean isCalculateNewVelocity = false;
    public boolean isCalculateNewAngle = false;

    double[] ValuesOfVelocity = {42.5, 47.5, 51.5, 51.5, 52.5, 57.5, 61, 62.5, 64.5}; // {45-47, 49-50, 56-57, 60, 61-62}
    double[] ValuesOfDistanceForVelocity = {1.061, 1.2058, 1.463, 1.8002, 1.8165, 2.0397, 2.3578, 2.4738, 2.6415};

    double[] ValuesOfAngle = {0, 1, 2, 3, 4, 5, 6, 7};
    double[] ValuesOfDistanceForAngle = {0, 1, 2, 3, 4, 5, 6, 7};

    public static double tag = 24;
    public boolean isShort, isLong = false;

    public AutoSniper(Turret turret, Shooter shooter) {
        tt = turret;
        sh = shooter;
    }

    public AutoSniper(Turret turret, Shooter shooter, Limelight limelight) {
        tt = turret;
        sh = shooter;
        ll = limelight;
    }

    public AutoSniper(Turret turret, Shooter shooter, Limelight limelight, Follower follower) {
        tt = turret;
        sh = shooter;
        ll = limelight;
        this.follower = follower;
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                goalX = 1;
                tag = 20;
                this.alliance = Alliance.BLUE;
                break;
            case RED:
                this.alliance = Alliance.RED;
                goalX = 138;
                tag = 24;
                break;
            case NONE:
                this.alliance = Alliance.NONE;
        }
    }

    public void continuousTurnTurretToGate(double x, double y, double angleOfDrivetrain) {
        if (AIMING_ACTIVE) {
            if (x <= 0) x = 1;
            if (x >= 144) x = 143;
//            if (!tt.isResetTurretPose) {
//                tt.setAimMethod(AimingMethod.TO_ZERO);
//            } else
            if ((ll.getGoalTag()[0] == 20 || ll.getGoalTag()[0] == 24)) {
                tt.setAimMethod(AimingMethod.CAMERA);
            } else {
                tt.setAimMethod(AimingMethod.LOCALIZATION);
                switch (alliance) {
                    case RED:
                        angleOfTurret = Math.toDegrees(Math.atan((goalY - (y + sY)) / (goalX - (x + sX))));
                        target = -(Math.toDegrees(angleOfDrivetrain) - angleOfTurret) + 90;
                        break;
                    case BLUE:
                        angleOfTurret = 180 - Math.toDegrees(Math.atan((goalY - (y + sY)) / ((x + sX) - goalX)));
                        target = -(Math.toDegrees(angleOfDrivetrain) - angleOfTurret) + 90;
                        break;
                    case NONE:
                        angleOfTurret = 0;
                        break;
                }

                target = tt.angleNormalising(target + targetBonus + driverTargetBonus);
                tt.turnByTarget(target);
                //tt.current = 0;

            }


        } else {
            tt.turnByTarget(0);
        }
    }

    public void continuousSetAngleByFormula(double servoPos) {
        double lastAngleOfAdjuster = convertServoPosToAngle(servoPos);
        if (D > 0) {
            isCalculateNewAngle = true;
            double var1 = Math.toDegrees(Math.atan((-l + Math.sqrt(D)) / (2 * a)));
            double var2 = Math.toDegrees(Math.atan((-l - Math.sqrt(D)) / (2 * a)));
            if (var1 >= 0 && Math.abs(var1 - lastAngleOfAdjuster) <= Math.abs(var2 - lastAngleOfAdjuster)) {
                angleOfAdjusterBeforeNormalising = var1;
                angleByFormulas = angleOfAdjusterBeforeNormalising;
            } else if (var2 >= 0 && Math.abs(var1 - lastAngleOfAdjuster) > Math.abs(var2 - lastAngleOfAdjuster)) {
                angleOfAdjusterBeforeNormalising = var2;
                angleByFormulas = angleOfAdjusterBeforeNormalising;
            } else {
                isCalculateNewAngle = false;
                angleOfAdjusterBeforeNormalising = lastAngleOfAdjuster;
                angleByFormulas = -1;
            }
        } else if (D == 0) {
            isCalculateNewAngle = true;
            angleOfAdjusterBeforeNormalising = Math.toDegrees(Math.atan((-l) / (2 * a)));
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

    public void continuousSetAngleByInterpol() {
        for (int i = 0; i < ValuesOfDistanceForAngle.length - 1; i++) {
            if (l > ValuesOfDistanceForAngle[7]) {
                l = ValuesOfDistanceForAngle[7];
            } else if (l < ValuesOfDistanceForAngle[0]) {
                l = ValuesOfDistanceForAngle[0];
            }
            if (ValuesOfDistanceForAngle[i] <= l && l <= ValuesOfDistanceForAngle[i + 1]) {
                v1 = ValuesOfAngle[i];
                v2 = ValuesOfAngle[i + 1];
                l1 = ValuesOfDistanceForAngle[i];
                l2 = ValuesOfDistanceForAngle[i + 1];

                targetAngle = v1 + (((l - l1) / (l2 - l1)) * (v2 - v1));
                sh.setAngleAdjuster(targetAngle);
            }
        }
    }

    public void continuousSetVelocityTargetByFormula(double servoPos, double lastAngularVelocity) {
        double angleOfAdjuster = Math.toRadians(convertServoPosToAngle(servoPos));
        if ((angleOfAdjusterBeforeNormalising <= MIN_ANGLE || angleOfAdjusterBeforeNormalising >= MAX_ANGLE)
                && (angleOfAdjusterBeforeNormalising != 0)) {
            isCalculateNewVelocity = true;
            targetVelo = cVTAV(Math.sqrt((g * Math.pow(l, 2) * (1 + Math.tan(angleOfAdjuster)))) / ((Math.tan(angleOfAdjuster) * l - cITM(z)) * 2));
            sh.setVelocityTarget(targetVelo + differenceVelocity);
        } else {
            isCalculateNewVelocity = false;
            sh.setVelocityTarget(lastAngularVelocity);
        }
    }

    public void continuousSetVelocityTargetByInterpol(double x, double y) {
        if ((y >= Math.abs(x - 72) + 60)) {
            for (int i = 0; i < ValuesOfDistanceForVelocity.length - 1; i++) {
                if (l > ValuesOfDistanceForVelocity[7]) {
                    l = ValuesOfDistanceForVelocity[7];
                } else if (l < ValuesOfDistanceForVelocity[0]) {
                    l = ValuesOfDistanceForVelocity[0];
                }
                if (ValuesOfDistanceForVelocity[i] <= l && l <= ValuesOfDistanceForVelocity[i + 1]) {
                    v1 = ValuesOfVelocity[i];
                    v2 = ValuesOfVelocity[i + 1];
                    l1 = ValuesOfDistanceForVelocity[i];
                    l2 = ValuesOfDistanceForVelocity[i + 1];

                    targetVelo = v1 + (((l - l1) / (l2 - l1)) * (v2 - v1));
                    sh.setVelocityTarget(targetVelo);
                }
            }
        } else {
            sh.setVelocityTarget(mainVelo);
        }
    }

    public boolean isSpinUp() {
        if (sh.getVelocityRPS() == 0) return false;
        return sh.getVelocityRPS() >= targetVelo - IS_SPIN_UP;
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
        return POS_FOR_MIN_ANGLE + ((angle - MIN_ANGLE) * (Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE) / Math.abs(MAX_ANGLE - MIN_ANGLE)));
    }

    public double convertServoPosToAngle(double servoPos) {
        return MIN_ANGLE + ((servoPos - POS_FOR_MIN_ANGLE) * (Math.abs(MAX_ANGLE - MIN_ANGLE) / Math.abs(POS_FOR_MIN_ANGLE - POS_FOR_MAX_ANGLE)));
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

    public void targetBonusUpdate(double y) {
        if (y < 55) targetBonus = 10;
        else targetBonus = 0;
    }

    public void trans(AimingMethod state) {
        this.state = state;
    }
}
