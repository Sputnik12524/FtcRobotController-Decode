package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.IOException;


@TeleOp(name = "TeleOpRR", group = "0")
@Disabled
@Config
public class TeleOpRoadRunner extends LinearOpMode {


    Shooter sh;
    Intake in;
    Turret tt;
    Transfer tr;
    Follower follower;
    AutoSniper as;
    Logger logger;
    Limelight ll;
    DriveTrain dt;


    boolean wroteLogger = true;
    boolean isPoseReset = false;
    boolean isInterpolActive = true;
    /// Intake

    /// Shooter
    boolean attentionControl = true;
    public double shortBonusVelocity = 0;
    public double longBonusVelocity = 0;
    double lastVelo = 0;


    @Override
    public void runOpMode() {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);
        follower = Constants.createFollower(hardwareMap);
        tr = new Transfer(this);
        ll = new Limelight(this);
        sh = new Shooter(this, follower);
        in = new Intake(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh, ll);
        dt = new DriveTrain(this);
        logger = new Logger("pospos");
        ll.startOrStopLL(false);


        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.heading));
            if (logger.al == Alliance.BLUE) {
                as.setAlliance(Alliance.BLUE);
            } else as.setAlliance(Alliance.RED);
            tt.ZeroRealPose = logger.turretPose;
        } catch (IOException | NullPointerException e) {
            tt.isResetTurretPose = true;
            isInterpolActive = false;
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
            as.setAlliance(Alliance.BLUE);
        }

        follower.update();

        if (wroteLogger)
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        sh.closeTunnel();
        tt.turnByTarget(0);
        tt.turretRegulator.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);


        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            as.continuousCalculateGeneralValues(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading(),
                    lastVelo
            );
            g1.update();
            g2.update();

            //------------------------------------- DRIVETRAIN
            follower.update();
            if (!attentionControl) tt.update(follower.getPose().getY());

            if (gamepad1.right_bumper) {
                dt.turnRightSlowMode();
            } else if (gamepad1.left_bumper) {
                dt.turnLeftSlowMode();
            } else {
                dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }

            //-------------------------------------- INTAKE

            if (g1.A.isPressed() && g1.A.getToggleState()) {
                in.rotateIn();
            } else if (g1.A.isPressed()) {
                in.rotateStop();
            }
            if (g1.B.isPressed() && g1.B.getToggleState()) {
                in.rotateOut();
            } else if (g1.B.isPressed()) {
                in.rotateStop();
            }

            //------------------------------------ SHOOTER
            if (!attentionControl) if (gamepad1.dpad_up) sh.canShoot = true;

            if (attentionControl || !isInterpolActive) {
                if (g1.Y.isPressed() && !g1.Y.getToggleState()) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW + longBonusVelocity);
                    sh.setLongThrowMode();
                    sh.shootByVelocity();
                } else if (g1.X.isPressed() && !g1.X.getToggleState()) {
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW + shortBonusVelocity);
                    sh.setShortThrowMode();
                    sh.shootByVelocity();
                } else if ((g1.Y.isPressed()) || (g1.X.isPressed())) {
                    sh.closeTunnel();
                    sh.shootStop();
                }
            } else {
                if(gamepad1.yWasPressed()){
                    isInterpolActive = false;
                    sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW + longBonusVelocity);
                    sh.setLongThrowMode();
                    sh.shootByVelocity();
                }

                if(gamepad1.xWasPressed()){
                    isInterpolActive = true;
                }

                if (isInterpolActive) {
                    as.setAngleByLocalisation(as.l, sh.getAngleAdjusterPos());
                    as.continuousSetVelocityTargetByInterpol(follower.getPose().getX(), follower.getPose().getY());
                    sh.shootByVelocity();
                    as.continuousSetAngleByFormula(
                            sh.getAngleAdjusterPos()
                    );
                }
            }


            if (g2.X.isPressed() && g2.X.getToggleState()) {
                isInterpolActive = false;
                sh.setVelocityTarget(0);
            } else if (g2.X.isPressed()) isInterpolActive = true;


            //---------------------------------------- TURRET
            if (!attentionControl)
                as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());



            /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

            if (g1.dpadRight.isPressed() && !attentionControl && g1.dpadRight.getToggleState()) {
                attentionControl = true;
                tt.turnByTarget(0);
            } else if (g1.dpadRight.isPressed() && !g1.dpadRight.getToggleState()) {
                attentionControl = false;
            }


            if (attentionControl) {
                if (gamepad1.dpad_up) {
                    sh.openTunnel();
                } else if (gamepad1.dpad_down) {
                    sh.closeTunnel();
                }

            }

            //------------------------------ POSE RESET
            if (g1.dpadUp.isPressed()) {
                tt.turnByTarget(0);
                isPoseReset = true;
                follower.setPose(new Pose(16, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.BLUE);
            }

            if (g1.dpadDown.isPressed()) {
                isPoseReset = true;
                tt.turnByTarget(0);
                follower.setPose(new Pose(129, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.RED);
            }

            if (gamepad2.leftBumperWasPressed()) {
                as.driverTargetBonus += 5;
            }
            if (gamepad2.rightBumperWasPressed()) {
                as.driverTargetBonus -= 5;
            }

            telemetry.addData("l", as.l);
            telemetry.addData("Velocity", sh.getVelocityRPS());

            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("isInterpol", isInterpolActive);
            telemetry.addData("Magnetic state", tt.isMagneting());
            telemetry.addData("InZone", sh.inZone());
            telemetry.addData("howMany", tr.howMany());
            // telemetry.addData("Позиция сброшена", isPoseReset);
            telemetry.addData("Alliance", as.alliance);
            telemetry.addData("Metod", tt.getAimMethod());
            telemetry.addData("TARGET", sh.velocityTarget / 28);
            telemetry.addLine(String.valueOf((int) (follower.getPose().getX())));
            telemetry.addLine(String.valueOf((int) follower.getPose().getY()));
            telemetry.addLine(String.valueOf((int) Math.toDegrees(follower.getHeading())));


            dashtele.addData("Target ", sh.velocityTarget / 28);
            dashtele.addData("Velocity shooter", sh.getVelocityRPS());
            dashtele.addData("ADJUSTER POS", sh.angleAdjuster.getPosition());
            dashtele.update();
            telemetry.update();
        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();

    }


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}