package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.util.Cycle;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.IOException;


@TeleOp(name = "TeleOpRR V3", group = "0")
@Config
public class TeleOpRoadRunnerV3 extends LinearOpMode {


    Shooter sh;
    Intake in;
    Turret tt;
    Transfer tr;
    Follower follower;
    AutoSniper as;
    Logger logger;
    Limelight ll;
    DriveTrain dt;
    Cycle cc;
    ElapsedTime time;
    ElapsedTime timer1;

    public static double p = 2;


    boolean wroteLogger = true;
    boolean isPoseReset = false;
    boolean isInterpolActive = true;
    boolean magnetic = false;
    boolean mState = true;
    /// Intake

    /// Shooter
    boolean attentionControl = false;
    public double shortBonusVelocity = 0;
    public double longBonusVelocity = 0;
    double lastVelo = 0;

    //intake
    boolean isRotateIn = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;
    boolean stateLSB = false;
    boolean stateB2 = false;

    //shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean isShootingShort = false;
    boolean isShootingMedium = false;
    boolean RSBState = false;
    double cycles;
    boolean slowMode = false;



    @Override
    public void runOpMode() {
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
        cc = new Cycle();
        ElapsedTime timerTelemetry = new ElapsedTime();
        time = new ElapsedTime();

        timer1 = new ElapsedTime();



        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.degrees));
            if (logger.al == Alliance.BLUE) {
                as.setAlliance(Alliance.BLUE);
            } else {
                as.setAlliance(Alliance.RED);
            }
        } catch (IOException | NullPointerException e) {
            isInterpolActive = false;
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
            as.setAlliance(Alliance.BLUE);
        }

        follower.update();

        if (wroteLogger)
            as.continuousTurnTurretToGate(follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading()
                    );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        sh.closeTunnel();
        tt.turnByTarget(0);
        tt.turretRegulator.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);


        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            long loopStart = System.nanoTime();

            //-------------------------------- DRIVETRAIN
            follower.update();
            artefactsControl();

            Pose pose = follower.getPose();
            double x = pose.getX();
            double y = pose.getY();
            double head = pose.getHeading();

            double main_input = -gamepad1.left_stick_y;
            double main = main_input + (1-main_input) * main_input * Math.abs(Math.pow(main_input,p-1));
            double side_input = gamepad1.left_stick_x;
            double side = side_input + (1-side_input) * side_input * Math.abs(Math.pow(side_input,p-1));
            double rotate_input = gamepad1.right_trigger - gamepad1.left_trigger;
            double rotate = rotate_input + (1-rotate_input) * rotate_input * Math.abs(Math.pow(rotate_input,p-1));


            dt.setMotorsPower(main, side, rotate);
            if (gamepad1.left_stick_button && !stateLSB && !slowMode) {
                DriveTrain.multiplier = 0.5;
                slowMode = true;
            } else if(ZOV) {
                DriveTrain.multiplier = 1;
                slowMode = true;
            }
            stateLSB = gamepad1.left_stick_button;

            //-------------------------------- INTAKE

            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
                isRotateOut = false;
            } else if ((gamepad1.b && !isRotateOut && !stateB1) || (gamepad2.b && !isRotateOut && !stateB2)) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
                sleep(120);
                in.rotateIn();
                isRotateOut = false;
                isRotateIn = true;
            }
            stateA1 = gamepad1.a;
            stateB1 = gamepad1.b;
            stateB2 = gamepad2.b;

            if (tt.isMagneting() && mState) {
                magnetic = true;
                mState = false;
            }
            if (gamepad2.aWasPressed()) mState = true;
            if (gamepad2.aWasPressed()) magnetic = false;

            //---------------------------------------- SHOOTER
//
//            if (gamepad1.y && !isShootingMedium && !stateY1) {
//                sh.setVelocityTarget(Shooter.VELOCITY_FOR_MEDIUM_THROW);
//                sh.setShortThrowMode();
//                sh.shootByVelocity();
//                isShootingMedium = true;
//                isShootingShort = false;
//            } else if (gamepad1.x && !isShootingShort && !stateX1) {
//                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
//                sh.setShortThrowMode();
//                sh.shootByVelocity();
//                isShootingMedium = false;
//                isShootingShort = true;
//            } else if ((gamepad1.y && !stateY1 && isShootingMedium) || (gamepad1.x && !stateX1 && isShootingShort)) {
//                sh.closeTunnel();
//                sh.shootStop();
//                isShootingMedium = false;
//                isShootingShort = false;
//            }
//            stateY1 = gamepad1.y;
//            stateX1 = gamepad1.x;

            as.continuousSetVelocityTargetByInterpol(
                    follower.getPose().getY()
            );
            sh.shootByVelocity();
            lastVel = sh.getVelocityRPS();

            as.setAngleByLocalisation(
                    as.l,
                    sh.getAngleAdjusterPos()
            );


            if (gamepad1.dpad_up) {
                sh.canShoot = true;
                in.rotateIn();
            }
            sh.coverSwitch();

            //-------------------------------- TURRET
            if (!attentionControl) {
                as.continuousTurnTurretToGate(x, y, head);
            } else {
                if ((ll.getGoalTag().get(0) == AutoSniper.tag) ) {
                    tt.setAimMethod(AimingMethod.CAMERA);
                } else {
                    tt.setAimMethod(AimingMethod.LOCALIZATION);
                    tt.turnByTarget(0);
                }
            }

            //--------------------------------- RESET AIM
            if (gamepad1.dpad_left) {
//                tt.turnByTarget(0);
           //     isPoseReset = true;
                follower.setPose(new Pose(16, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.BLUE);
            }

            if (gamepad1.dpad_right) {
//                tt.turnByTarget(0);
           //     isPoseReset = true;
                follower.setPose(new Pose(129, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.RED);
            }

            /// EXTRA MANUAL CONTROL

            if (gamepad1.right_stick_button && !RSBState && !attentionControl) {
                attentionControl = true;
            } else if (gamepad1.right_stick_button && !RSBState && attentionControl) {
                attentionControl = false;
            }
            RSBState = gamepad1.right_stick_button;


            as.continuousCalculateGeneralValues(
                    x,y,head,
                    lastVelo
            );
            lastVelo = sh.getVelocityRPS();
            cc.update();

            long loopTimeNs = System.nanoTime() - loopStart;
            double loopMs = loopTimeNs / 1e6;

   //           telemetry.addData("Loop ms", loopMs);
            telemetry.addData("l", as.l);
//            telemetry.addData("All time", cc.getAll());
//            telemetry.addData("Average Cycle", cc.getAverage());
//            telemetry.addData("MAX Cycle", cc.getMax());
//
          //   telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
//            telemetry.addData("isInterpol", isInterpolActive
            telemetry.addData("Magnetic state", tt.isMagneting());
//            telemetry.addData("InZone", sh.inZone());
//            telemetry.addData("howMany", tr.howMany());
//            //telemetry.addData("Позиция сброшена", isPoseReset);
            telemetry.addData("Alliance", as.alliance);
            telemetry.addData("AimMethod", tt.getAimMethod());
//            telemetry.addLine(String.valueOf((int) (x)));
//            telemetry.addLine(String.valueOf((int) y));
//            telemetry.addLine(String.valueOf((int) Math.toDegrees(head)));
//
            telemetry.addData("error TT", tt.error);
            telemetry.addData("target TT", tt.target);
           // telemetry.addData("Target", sh.velocityTarget / 28);
//            telemetry.addData("Velocity shooter", sh.getVelocityRPS());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
          //  dashtele.update();
            telemetry.update();

        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();


    }

    void ampsUpdate() {
        if (time.milliseconds() > 100) {
            sh.voltageUP += sh.getUpAmps();
            sh.voltageLOW += sh.getLowAmps();
            in.voltage += in.getAmps();
            tt.voltage += tt.getAmps();
            time.reset();
        }
    }

    void artefactsControl() {
        if (tr.howMany() > 3 && timer1.milliseconds() > 300) {
            in.rotateStop();
            timer1.reset();
        }
    }


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}