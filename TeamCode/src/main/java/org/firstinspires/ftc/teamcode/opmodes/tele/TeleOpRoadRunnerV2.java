package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFSM;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Paths;


@TeleOp(name = "TeleOpRR_V2", group = "0")
@Disabled
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    Paths paths;
    Shooter sh;
    Intake in;
    Turret tt;
    Follower follower;
    AutoSniper as;
    Logger logger;
    Limelight ll;
    ElapsedTime timer;
    ElapsedTime autoStTimer;
    ElapsedTime autoTimer;
    DriveTrain dt;
    AutoFSM af;
    Telemetry dashtele;
    FtcDashboard dashboard;

    boolean wroteLogger = true;
    boolean isPoseReset = false;
    boolean attentionControl = false;
    double lastVelo = 0;

    @Override
    public void runOpMode() {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        logger = new Logger("pospos");
        ll = new Limelight(this);
        sh = new Shooter(this, follower);
        in = new Intake(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh, ll);
        dt = new DriveTrain(this);
        paths = new Paths(follower);
        af = new AutoFSM(follower, sh, ll, in, tt, logger, as, paths);

        dashboard = FtcDashboard.getInstance();
        dashtele = dashboard.getTelemetry();
        timer = new ElapsedTime();
        autoTimer = new ElapsedTime();
        autoStTimer = new ElapsedTime();


        wroteLogger = af.wroteLogger;
        attentionControl = af.wroteLogger;

        if (wroteLogger)
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

        ll.startOrStopLL(false);
        sh.closeTunnel();
        tt.turnByTarget(0);
        tt.turretRegulator.start();
        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            if (af.mode == AutoFSM.MODE.DRIVER){
                driverUpdate(g1, g2);
            }
            else {
                    af.update();
                if (af.complete) {
                    if(follower.isBusy()) follower.breakFollowing();
                    af.mode = AutoFSM.MODE.DRIVER;
                    af.complete = false;
                }
            }
            updateAutomatic(as.l, sh.getAngleAdjusterPos());
            checkFollower();
            sh.update();
            updateTelemetry();
            ampsUpdate();
        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }


    public void driverUpdate(GamepadManager g1, GamepadManager g2) {
        g1.update();
        g2.update();

        /// =====================AUTO===============================///

        if (gamepad2.bWasPressed()) {
            if(canMove())af.setAuto(AutoFSM.AUTO.PARK);
        }
        if (gamepad2.aWasPressed()) { //кнопки переписать
            if(canMove())af.setAuto(AutoFSM.AUTO.GOAL);
        }
        if (gamepad2.xWasPressed()) {
            if(canMove())af.setAuto(AutoFSM.AUTO.HUMAN);
        }

        /// =====================DRIVE TRAIN===============================///

        if (g1.rightBumper.isPressed()) {
            dt.turnRightSlowMode();
        } else if (g1.leftBumper.isPressed()) {
            dt.turnLeftSlowMode();
        } else {
            dt.setMotorsPower(-g1.leftStickY, g1.leftStickX, g1.rightTrigger - g1.leftTrigger); //проверить ездит ли
        }

        /// =====================INTAKE===============================///

        if (g1.A.isPressed() && g1.A.getToggleState()) {
            in.rotateIn();
        } else if (g1.A.isPressed() && !g1.A.getToggleState()) {
            in.rotateStop();
        }

        if (g1.B.isPressed() && g1.B.getToggleState()) {
            in.rotateOut();
        } else if (g1.B.isPressed() && !g1.B.getToggleState()) {
            in.rotateStop();
        }

        /// =====================SHOOTER===============================///

        if (!attentionControl) if (gamepad1.dpad_up) sh.isCanShoot = true;

        if (g1.X.isPressed() && !g1.X.getToggleState()) {
            sh.transit(Shooter.ShStates.SPINNING);
        } else if (g1.X.isPressed() && g1.X.getToggleState()) {
            sh.transit(Shooter.ShStates.STOP);
        }

        /// =====================TURRET===============================///
        if (!attentionControl)
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

        if (gamepad2.leftBumperWasPressed()) {
            as.targetBonus += 5;
        }
        if (gamepad2.rightBumperWasPressed()) {
            as.targetBonus -= 5;
        }

        /// =====================ATTENTION CONTROL===============================///

        if (g1.rightStickButton.isPressed() && g1.rightStickButton.getToggleState()) {
            sh.mode = Shooter.MODE.MANUAL;
            attentionControl = true;
            sh.setManual(true);
            tt.turnByTarget(0);
        } else if (g1.rightStickButton.isPressed() && !g1.rightStickButton.getToggleState()) {
            sh.mode = Shooter.MODE.AUTO;
            attentionControl = false;
            sh.setManual(false);
        }

        if (attentionControl) {
            if (g1.dpadUp.isPressed()) {
                sh.openTunnel();
            } else if (g1.dpadDown.isPressed()) {
                sh.closeTunnel();
            }
        }

        /// =====================POSE RESET===============================///

        if (g2.dpadLeft.isPressed()) {
            resetPose(Alliance.BLUE);
        }

        if (g2.dpadRight.isPressed()) {
            resetPose(Alliance.RED);
        }

        /// =====================ЗАЩИТА😎==============================///

        if (g1.dpadUp.isPressed() && af.mode == AutoFSM.MODE.AUTO) af.mode = AutoFSM.MODE.DRIVER;
    }

    void resetPose(Alliance alliance) {
        isPoseReset = true;
        if(alliance == Alliance.BLUE) follower.setPose(new Pose(16, 80, Math.toRadians(90)));
        else follower.setPose(new Pose(129, 80, Math.toRadians(90)));
        as.setAlliance(alliance);
    }

    void ampsUpdate() {
        if (timer.milliseconds() > 100) {
            sh.voltageUP += sh.getUpAmps();
            sh.voltageLOW += sh.getLowAmps();
            in.voltage += in.getAmps();
            tt.voltage += tt.getAmps();
            timer.reset();
        }
    }

    void updateAutomatic(double l, double pose) {
        if (attentionControl) return;
        as.setAngleByLocalisation(l, pose);
        as.continuousSetVelocityTargetByInterpol(follower.getPose().getX(), follower.getPose().getY());
        sh.shootByVelocity();
        as.continuousSetAngleByFormula(
                sh.getAngleAdjusterPos()
        );
    }

    boolean canMove(){
        return  (!follower.isBusy() && af.mode == AutoFSM.MODE.DRIVER);
    }
    void checkFollower(){
        if(follower.getPose().getX() == 0 && follower.getPose().getY() == 0) attentionControl = true;
    }

    void updateTelemetry() {
        telemetry.addData("TeleState", af.mode);
        telemetry.addData("autoState", af.autoState);
        telemetry.addData("Shooter LOW AMPS", sh.getLowAmps());
        telemetry.addData("Shooter UP AMPS", sh.getUpAmps());
        telemetry.addData("Intake AMPS", in.getAmps());
        telemetry.addData("Turret AMPS", tt.getAmps());
        telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
        telemetry.addData("Velocity", sh.getVelocityRPS());
        telemetry.addData("InZone", sh.inZone());
        // telemetry.addData("Позиция сброшена", isPoseReset);
        telemetry.addData("Alliance", as.alliance);
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


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}