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

    boolean aState = false;
    boolean angleState = false;

    boolean RSBState = false;
    boolean constVeloState = false;



    @Override
    public void runOpMode() {
        tt = new Turret(this);
        sh = new Shooter(this);
        as = new AutoSniper(tt, sh);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();

        tt.turretRegulator.start();

        as.setAlliance(Alliance.RED);

        sh.setVelocityTarget(0);
        sh.setLongThrowMode();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();


            //---------------------------------------------- TURRET

            if (gamepad1.x && !xState && !turretState) {
                turretState = true;
            } else if (gamepad1.x && !xState && turretState) {
                turretState = false;
            }
            xState = gamepad1.x;

            if (turretState) {
                as.continuousTurnTurretToGate( follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

                telemetry.addLine("TURRET TELEMETRY:");
                telemetry.addData("target", tt.target);
                telemetry.addData("error", tt.error);
                telemetry.addData("target FROM AutoSniper", as.target);
                telemetry.addData("angleOfTurret (отн. поля)", as.angleOfTurret);
                telemetry.addData("x:", follower.getPose().getX());
                telemetry.addData("y:", follower.getPose().getY());
                telemetry.addData("head", follower.getPose().getHeading());
            } else {
                telemetry.addLine("TURRET is stopped");
                tt.turnByTarget(0);
                turretState = false;
            }


            //---------------------------------------------- VELOCITY

            if (gamepad1.b && !bState && !veloState) {
                veloState = true;
            } else if (gamepad1.b && !bState && veloState) {
                veloState = false;
            }
            bState = gamepad1.b;

            if (gamepad1.right_stick_button && !RSBState && !constVeloState) {
                constVeloState = true;
            } else if (gamepad1.right_stick_button && !RSBState && constVeloState) {
                constVeloState = false;
            }
            RSBState = gamepad1.right_stick_button;

            sh.shootByVelocity();
            if (veloState && !constVeloState) {
                as.continuousSetVelocityTarget(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getHeading(),
                        sh.getAngleAdjusterPos(),
                        lastVelo
                );
                telemetry.addLine("VELOCITY TELEMETRY:");
                telemetry.addData("targetVelocity", as.targetVeloForArtifact);
                telemetry.addData("realVelocity", sh.getVelocityRPS());
            } else if (constVeloState && !veloState) {
                sh.setVelocityTarget(50);
                telemetry.addLine("VELOCITY IS CONST");
            } else {
                telemetry.addLine("VELOCITY is stopped");
                sh.setVelocityTarget(0);
            }
            lastVelo = sh.getVelocityRPS();


            //---------------------------------------------- ANGLE

            if (gamepad1.a && !aState && !angleState) {
                angleState = true;
            } else if (gamepad1.a && !aState && angleState) {
                angleState = false;
            }
            aState = gamepad1.a;

            if (angleState) {
                as.continuousSetAngle(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getHeading(),
                        sh.getAngleAdjusterPos(),
                        lastVelo
                );
                telemetry.addLine("ANGLE TELEMETRY:");
                telemetry.addData("targetAngle", as.angleOfAdjuster);
                telemetry.addData("realAngle", as.convertServoPosToAngle(sh.getAngleAdjusterPos()));
                telemetry.addData("ServoPos", sh.getAngleAdjusterPos());
                telemetry.addData("NON NORMALISING ANGLE", as.angleOfAdjusterBeforeNormalising);
            } else {
                telemetry.addLine("Set long throw ANGLE");
                sh.setAngleAdjuster(Shooter.POS_LONG_THROW);
            }


            //---------------------------------------------- TELEMETRY

            telemetry.addData("x:", follower.getPose().getX());
            telemetry.addData("y:", follower.getPose().getY());
            telemetry.addData("head", follower.getPose().getHeading());
            telemetry.update();
        }

        tt.turretRegulator.interrupt();

    }
}
