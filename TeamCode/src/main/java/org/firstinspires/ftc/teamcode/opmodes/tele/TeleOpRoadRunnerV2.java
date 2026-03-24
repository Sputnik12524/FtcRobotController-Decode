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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFSM;
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Paths;

import java.io.IOException;


@TeleOp(name = "TeleOpRR_V2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    enum MMODE {MANUAL, AUTO}

    MMODE mod = MMODE.AUTO;


    Paths paths;
    Shooter sh;
    Intake in;
    Turret tt;
    Transfer tr;
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

    @Override
    public void runOpMode() {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        logger = new Logger("pospos");
        tr = new Transfer(this);
        ll = new Limelight(this);
        sh = new Shooter(this, follower, tr);
        in = new Intake(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh);
        dt = new DriveTrain(this);
        paths = new Paths(follower);
        af = new AutoFSM(follower, tr, sh, ll, in, tt, logger, as, paths);

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
            if (af.mode == AutoFSM.MODE.DRIVER) driverUpdate(g1, g2);
            else {
                follower.update();
                af.update();
                if (af.complete) {
                    af.mode = AutoFSM.MODE.DRIVER;
                    af.complete = false;
                }
            }
            sh.update();
            updateTelemetry();
        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }




    public void resetPose(Alliance alliance) {
        isPoseReset = true;          //tt.turnByTarget(0); //пересмотреть
        follower.setPose(new Pose(11, 7, 0));
        as.setAlliance(alliance);
    }

    public void updateTelemetry() {
        telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
        telemetry.addData("Velocity", sh.getVelocityRPS());
        telemetry.addData("InZone", sh.inZone());
        telemetry.addData("howMany", tr.howMany());
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

    public void driverUpdate(GamepadManager g1, GamepadManager g2) {
        g1.update();
        g2.update();

        /// =====================AUTO===============================///

        if (gamepad2.bWasPressed()) {
            af.setAuto(AutoFSM.AUTO.PARK);
        }
        if (gamepad2.aWasPressed()) { //кнопки переписать
            af.setAuto(AutoFSM.AUTO.GOAL);
        }
        if (gamepad2.xWasPressed()) {
            af.setAuto(AutoFSM.AUTO.HUMAN);
        }

        /// =====================DRIVE TRAIN===============================///

        if (gamepad1.right_bumper) {
            dt.turnRightSlowMode();
        } else if (gamepad1.left_bumper) {
            dt.turnLeftSlowMode();
        } else {
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
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

        if (g1.Y.isPressed() && g1.Y.getToggleState()) {
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

        if (g1.dpadRight.isPressed() && g1.dpadRight.getToggleState()) {
            sh.mode = Shooter.MODE.MANUAL;
            attentionControl = true;
            sh.setManual(true);
            tt.turnByTarget(0);
        } else if (g1.dpadRight.isPressed() && !g1.dpadRight.getToggleState()) {
            sh.mode = Shooter.MODE.AUTO;
            attentionControl = false;
            sh.setManual(false);
        }

        if (attentionControl) {
            if (gamepad1.dpad_up) {
                sh.openTunnel();
            } else if (gamepad1.dpad_down) {
                sh.closeTunnel();
            }
        }

        /// =====================POSE RESET===============================///

        if (g2.dpadUp.isPressed()) {
            resetPose(Alliance.BLUE);
        }

        if (g2.dpadDown.isPressed()) {
            resetPose(Alliance.RED);
        }

        /// =====================ЗАЩИТА😎===============================///

        if (g1.dpadUp.isPressed() && af.mode == AutoFSM.MODE.AUTO) af.mode = AutoFSM.MODE.DRIVER;
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}