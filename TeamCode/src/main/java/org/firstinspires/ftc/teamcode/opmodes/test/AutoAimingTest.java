package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;

@Config
@TeleOp(name = "TEST AutoAiming / turret, velocity, angle", group = "1")
public class AutoAimingTest extends LinearOpMode {

    Follower follower;
    AutoSniper as;
    Turret tt;
    Shooter sh;
    Limelight ll;

    double lastVelo = 0;

    boolean xState = false;
    boolean turretState = false;

    boolean bState = false;
    boolean veloState = false;
    boolean angleLocState = false;

    boolean aState = false;
    boolean angleContState = false;

    boolean RSBState = false;
    boolean constVeloState = false;

    boolean yState = false;
    boolean interpolState = false;

    boolean LBState = false;
    boolean generalState = false;



    @Override
    public void runOpMode() {
        ll = new Limelight(this);
        tt = new Turret(this);
        sh = new Shooter(this);
        as = new AutoSniper(tt, sh, ll);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();

        tt.turretRegulator.start();

        as.setAlliance(Alliance.BLUE);
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        sh.setVelocityTarget(0);


        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            //---------------------------------------------- VALUES

            follower.update();
            as.continuousCalculateGeneralValues(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading(),
                    lastVelo
            );


            //---------------------------------------------- TURRET

            if (gamepad1.x && !xState && !turretState) {
                as.enableAutoTurretAiming(true);
                turretState = true;
            } else if (gamepad1.x && !xState && turretState) {
                    as.enableAutoTurretAiming(false);
                turretState = false;
            }
            xState = gamepad1.x;

            if (turretState) {
                as.continuousTurnTurretToGate(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getHeading()
                );

                telemetry.addLine("TURRET TELEMETRY:");
                telemetry.addData("target", tt.target);
                telemetry.addData("error", tt.error);
                telemetry.addData("target FROM AutoSniper", as.target);
                telemetry.addData("angleOfTurret (отн. поля)", as.angleOfTurret);
                telemetry.addData("Aiming method", tt.getAimMethod());
            } else {
                telemetry.addLine("TURRET is stopped (put X)");
                tt.turnByTarget(0);
                turretState = false;
            }


            //---------------------------------------------- VELOCITY

//
            as.continuousSetVelocityTargetByInterpol(
                    follower.getPose().getY()
            );
            sh.shootByVelocity();
            telemetry.addLine("VELOCITY TELEMETRY (INTERPOL):");
                telemetry.addData("targetVelocity", as.targetVeloForArtifact);
                telemetry.addData("realVelocity", sh.getVelocityRPS());
            lastVelo = sh.getVelocityRPS();

//            telemetry.addLine("");
//            if (veloState && !constVeloState && !interpolState) {
//                as.continuousSetVelocityTargetByFormula(
//                        sh.getAngleAdjusterPos(),
//                        lastVelo
//                );
//                telemetry.addLine("VELOCITY TELEMETRY (FORMULA):");
//                telemetry.addData("targetVelocity", as.targetVeloForArtifact);
//                telemetry.addData("realVelocity", sh.getVelocityRPS());
//                telemetry.addData("isCalculateNewVelocity?", as.isCalculateNewVelocity);
//            } else if (constVeloState && !veloState && !interpolState) {
//                sh.setVelocityTarget(50);
//                telemetry.addLine("VELOCITY IS CONST");
//            } else if (interpolState && !constVeloState && !veloState) {
//                as.continuousSetVelocityTargetByInterpol();
//                telemetry.addLine("VELOCITY TELEMETRY (INTERPOL):");
//                telemetry.addData("targetVelocity", as.targetVeloForArtifact);
//                telemetry.addData("realVelocity", sh.getVelocityRPS());
//            } else {
//                telemetry.addLine("VELOCITY is stopped (put Y, B or RS)");
//                sh.setVelocityTarget(0);
//            }
//            lastVelo = sh.getVelocityRPS();


            //---------------------------------------------- ANGLE

            if (gamepad1.a && !aState && !angleContState) {
                angleContState = true;
            } else if (gamepad1.a && !aState && angleContState) {
                angleContState = false;
            }
            aState = gamepad1.a;

            telemetry.addLine("");
            if (angleContState && !angleLocState) {
                as.continuousSetAngleByFormula(
                        sh.getAngleAdjusterPos()
                );
                telemetry.addLine("ANGLE TELEMETRY (FORMULA):");
                telemetry.addData("targetAngle", as.angleOfAdjuster);
                telemetry.addData("realAngle", as.convertServoPosToAngle(sh.getAngleAdjusterPos()));
                telemetry.addData("ServoPos", sh.getAngleAdjusterPos());
                telemetry.addData("NON NORMALISING ANGLE", as.angleOfAdjusterBeforeNormalising);
                telemetry.addData("BY FORMULAS", as.angleByFormulas);
                telemetry.addData("isCalculateNewAngle?", as.isCalculateNewAngle);
            } else if (angleLocState && !angleContState) {
                as.setAngleByLocalisation(
                        as.l,
                        sh.getAngleAdjusterPos()
                );
                telemetry.addLine("ANGLE TELEMETRY (INTERPOL):");
                telemetry.addData("targetAngle", as.angleOfAdjuster);
                telemetry.addData("Is long throw?", as.isLong);
                telemetry.addData("Is short throw?", as.isShort);
            } else {
                telemetry.addLine("default long ANGLE (put Y or A)");
                sh.setLongThrowMode();
            }


            //---------------------------------------------- TELEMETRY

            if (gamepad1.left_bumper && !LBState && !generalState) {
                generalState = true;
            } else if (gamepad1.left_bumper && !LBState && generalState) {
                generalState = false;
            }
            LBState = gamepad1.left_bumper;

            telemetry.addLine("");
            if (generalState) {
                telemetry.addLine("GENERAL VALUES:");
                telemetry.addData("x:", follower.getPose().getX());
                telemetry.addData("y:", follower.getPose().getY());
                telemetry.addData("head (deg)", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addData("head (rad)", follower.getPose().getHeading());
                telemetry.addData("sX", as.sX);
                telemetry.addData("D", as.D);
                telemetry.addData("a", as.a);
                telemetry.addData("l (b)", as.l);
                telemetry.addData("c", as.c);
            } else {
                telemetry.addLine("GENERAL VALUES is disabled (put LB)");
            }
            telemetry.update();
        }

        tt.turretRegulator.interrupt();
    }
}
