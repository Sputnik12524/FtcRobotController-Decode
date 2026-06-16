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
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.IOException;


@TeleOp(name = "TeleOpRR V3", group = "0")
@Config
public class TeleOpRoadRunnerV3 extends LinearOpMode {
    enum IN_STATES {DEFAULT, WAIT, IN, OUT}

    IN_STATES state = IN_STATES.DEFAULT;

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
    ElapsedTime timerTurret, timerCalc, timer, timer1, time, timerVelo;
    GamepadManager g1;
    GamepadManager g2;

    public static double turret_kDC = 0;
    public static double turret_kIC = 0;
    public static double turret_kPC = 0.0189;
    public static double turret_kDL = 0.02;
    public static double turret_kIL = 0;
    public static double turret_kPL = 0.021;

    boolean wroteLogger = true;
    boolean isPoseReset = false;
    boolean isInterpolActive = true;
    boolean magnetic = false;
    boolean mState = true;
    /// Intake

    /// Shooter
    boolean attentionControl = true;
    public double shortBonusVelocity = 0;
    public double longBonusVelocity = 0;
    public double lastVelo = 0;

    //intake
    boolean isRotateIn = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;
    boolean stateLSB = false;
    boolean stateB2 = false;
    final double WAIT_TIME = 150;

    //shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean isShootingShort = false;
    boolean isShootingMedium = false;
    boolean RSBState = false;
    double cycles;
    boolean slowMode = false;
    Pose pose;
    double x, y, head;


    @Override
    public void runOpMode() {
        g1 = new GamepadManager(gamepad1);
        g2 = new GamepadManager(gamepad2);
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

        InterpolAndAimingThread interpolAndAimingThread = new InterpolAndAimingThread();
        DTFormulasThread dtFormulasThread = new DTFormulasThread();

        timerTurret = new ElapsedTime();
        timer = new ElapsedTime();
        timer1 = new ElapsedTime();
        timerCalc = new ElapsedTime();
        timerVelo = new ElapsedTime();
        time = new ElapsedTime();

        ll.update();

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
        ll.lt.start();
        tt.setAimMethod(AimingMethod.LOCALIZATION);

        waitForStart();
        //interpolAndAimingThread.start();
        dtFormulasThread.start();

        while (opModeIsActive()) {
            ll.update();
            long loopStart = System.nanoTime();
            tt.tuneTurretPID(turret_kPL,turret_kIL,turret_kDL,turret_kPC,turret_kDC);

            as.continuousCalculateGeneralValues(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading(),
                    lastVelo
            );

//            //-------------------------------- DRIVETRAIN

            follower.update();
            pose = follower.getPose();
            x = pose.getX();
            y = pose.getY();
            head = pose.getHeading();
            inUpdate();
            g1.update();
            g2.update();

            //-------------------------------- BACKLIGHT

            if (as.isSpinUp() && !sh.ifNotInLaunchZoneGoal() && !sh.ifNotInLaunchZoneHuman()) {
                sh.turnOnLight();
            } else {
                sh.turnOffLight();
            }

            //-------------------------------- INTAKE

            if (tt.isMagneting() && mState) {
                magnetic = true;
                mState = false;
            }
            if (gamepad2.aWasPressed()) mState = true;
            if (gamepad2.aWasPressed()) magnetic = false;

            //---------------------------------------- SHOOTER

            sh.setVelocityTarget(30);
            sh.shootByVelocity();
            //lastVelo = sh.getVelocityRPS();23

            sh.coverSwitch();

            //-------------------------------- TURRET
            as.continuousTurnTurretToGate(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading()
            );

            //--------------------------------- RESET AIM
            if (gamepad1.dpad_left) {
                tt.turnByTarget(0);
                isPoseReset = true;
                follower.setPose(new Pose(16, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.BLUE);
            }

            if (gamepad1.dpad_right) {
                tt.turnByTarget(0);
                isPoseReset = true;
                follower.setPose(new Pose(129, 80, Math.toRadians(90)));
                as.setAlliance(Alliance.RED);
            }
            if (gamepad1.leftBumperWasPressed()) {
                sh.canShoot = true;
            }

            /// EXTRA MANUAL CONTROL

            if (gamepad1.right_stick_button && !RSBState && !attentionControl) {
                attentionControl = true;
            } else if (gamepad1.right_stick_button && !RSBState && attentionControl) {
                attentionControl = false;
            }
            RSBState = gamepad1.right_stick_button;


            long loopTimeNs = System.nanoTime() - loopStart;
            double loopMs = loopTimeNs / 1e6;

            telemetry.addData("Turret localization kP", tt.getLocalizationCoefficients()[0]);
            telemetry.addData("turret kp", turret_kPL);
            telemetry.addData("kI", tt.getLocalizationCoefficients()[1]);
            telemetry.addData("kD", tt.getLocalizationCoefficients()[2]);
            telemetry.addData("Turret camera kP", tt.getCameraCoefficients()[0]);
            telemetry.addData("Turret camera kD", tt.getCameraCoefficients()[1]);


//            telemetry.addData("Target", as.targetVelo);
//            telemetry.addData("Velocity shooter", sh.getVelocityRPS());

            /*telemetry.addData("Loop ms", loopMs);
            telemetry.addData("l", as.l);
            telemetry.addData("All time", cc.getAll());
            telemetry.addData("Average Cycle", cc.getAverage());
            telemetry.addData("MAX Cycle", cc.getMax());
            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("isInterpol", isInterpolActive
                    telemetry.addData("Magnetic state", tt.isMagneting());
            telemetry.addData("InZone", sh.inZone());
            telemetry.addData("howMany", tr.howMany());
            telemetry.addData("Позиция сброшена", isPoseReset);
            telemetry.addData("Alliance", as.alliance);
            telemetry.addData("AimMethod", tt.getAimMethod());
            telemetry.addLine(String.valueOf((int) (x)));
            telemetry.addLine(String.valueOf((int) y));
            telemetry.addLine(String.valueOf((int) Math.toDegrees(head)));
            telemetry.addData("error TT", tt.error);
            telemetry.addData("target TT", tt.target);
            telemetry.addData("target TT AS", as.target);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            dashtele.update();*/
            telemetry.update();

        }
        ll.startOrStopLL(true);
        interpolAndAimingThread.interrupt();
        ll.lt.interrupt();
        dtFormulasThread.interrupt();
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

    void setInState(IN_STATES state) {
        timer.reset();
        this.state = state;
    }

    void artefactsControl() {
        if (tr.howMany() == 3 && timer1.milliseconds() > 300) {
            in.rotateStop();
            timer1.reset();
        }
    }


    void inUpdate() {
        switch (state) {
            case DEFAULT:
                if (g1.A.isPressed() && g1.A.getToggleState()) {
                    setInState(IN_STATES.IN);
                } else if (g1.B.isPressed()) {
                    setInState(IN_STATES.OUT);
                } else in.rotateStop();
                break;
            case IN:
                in.rotateIn();
                if (g1.A.isPressed() && !g1.A.getToggleState()) {
                    setInState(IN_STATES.DEFAULT);
                } else if (g1.B.isPressed()) {
                    setInState(IN_STATES.WAIT);
                }
                break;
            case OUT:
                in.rotateOut();
                setInState(IN_STATES.WAIT);
                break;
            case WAIT:
                in.rotateOut();
                if (timer.milliseconds() > WAIT_TIME) setInState(IN_STATES.DEFAULT);
                break;
        }
    }


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

    public class InterpolAndAimingThread extends Thread {
        @Override
        public void run() {
            while (!isInterrupted()) {
                if (!attentionControl) {
                    as.continuousSetVelocityTargetByInterpol(x, y);
                    as.setAngleByLocalisation(as.l, sh.getAngleAdjusterPos());
                    as.continuousCalculateGeneralValues(x, y, head, lastVelo);
                } else {
                    if (gamepad1.x && !isShootingShort && !stateX1) {
                        sh.setVelocityTarget(-Shooter.VELOCITY_FOR_SHORT_THROW);
                        sh.setShortThrowMode();
                        sh.shootByVelocity();
                        isShootingMedium = false;
                        isShootingShort = true;
                    } else if (gamepad1.x && !stateX1 && isShootingShort) {
                        sh.closeTunnel();
                        sh.shootStop();
                        isShootingMedium = false;
                    }
                }

                if (!attentionControl) {
                    as.continuousTurnTurretToGate(x, y, head);
                } else {
                    tt.setAimMethod(AimingMethod.LOCALIZATION);
                    tt.turnByTarget(0);
                }

            }
        }
    }

    public class DTFormulasThread extends Thread {
        @Override
        public void run() {
            while (!isInterrupted()) {
                dt.setMotorsPowerNonLinear(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }
        }
    }
}