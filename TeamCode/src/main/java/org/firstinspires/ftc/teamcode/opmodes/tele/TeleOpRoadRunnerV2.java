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
import org.firstinspires.ftc.teamcode.util.AutoSniper;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Paths;

import java.io.IOException;


@TeleOp(name = "TeleOpRR_V2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    enum MODULES {INTAKE, SHOOTER, TURRET, DRIVE_TRAIN}

    MODULES modules;

    enum MMODE {MANUAL, AUTO}

    MMODE mod = MMODE.AUTO;

    enum AUTO {DEF, B_PARK, R_PARK, B_HUMAN, R_HUMAN, B_GOAL, R_GOAL, INIT_PARK, INIT_HUMAN, INIT_GOAL}

    AUTO auto = AUTO.DEF;

    enum AutoStates {DEF, MOVE, CHECK, SHOOT, INIT}

    AutoStates autoState = AutoStates.DEF;

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

    enum MODE {AUTO, DRIVER}

    MODE mode = MODE.DRIVER;

    double target; //переместить
    boolean wroteLogger = true;
    boolean isPoseReset = false;
    Alliance alliance;
    boolean complete = false;
    /// Intake

    /// Shooter
    boolean attentionControl = false;

    @Override
    public void runOpMode() {
        GamepadManager g1 = new GamepadManager(gamepad1);
        GamepadManager g2 = new GamepadManager(gamepad2);
        follower = Constants.createFollower(hardwareMap);
        tr = new Transfer(this);
        ll = new Limelight(this);
        sh = new Shooter(this, follower, tr);
        in = new Intake(this);
        tt = new Turret(this, ll);
        as = new AutoSniper(tt, sh);
        dt = new DriveTrain(this);
        logger = new Logger("pospos");
        paths = new Paths(follower);
        timer = new ElapsedTime();
        autoTimer = new ElapsedTime();
        autoStTimer = new ElapsedTime();


        ll.startOrStopLL(false);

        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.degrees));
            if (logger.al == Alliance.BLUE) {
                alliance = Alliance.BLUE;
                as.setAlliance(Alliance.BLUE);
            } else {
                as.setAlliance(Alliance.RED);
                alliance = Alliance.RED;
            }
        } catch (IOException | NullPointerException e) {
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            attentionControl = true;
            as.setAlliance(Alliance.NONE);
        }


        if (wroteLogger)
            as.continuousTurnTurretToGate(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        sh.closeTunnel();
        tt.turnByTarget(0);
        tt.turretRegulator.start();
        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            if (mode == MODE.DRIVER) driverUpdate(g1, g2);
            else {
                follower.update();
                autoStatesUpdate();
                autoUpdate();
                if (complete) {
                    mode = MODE.DRIVER;
                    complete = false;
                }
            }
            sh.update();


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
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }


    public void autoStatesUpdate() {
        switch (autoState) {
            case DEF:
                break;

            case INIT:
                sh.setVelocityTarget(target);
                sh.closeTunnel();
                in.rotateStop();
                setAutoState(AutoStates.MOVE);
                break;

            case MOVE:
                if (!follower.isBusy()) setAutoState(AutoStates.CHECK);
                break;

            case CHECK:
                if (sh.isSpinUp()) setAutoState(AutoStates.SHOOT);
                break;

            case SHOOT:
                in.rotateIn();
                sh.openTunnel();
                if (autoStTimer.milliseconds() > 400) {
                    setAutoState(AutoStates.DEF);
                    setAuto(AUTO.DEF);
                    complete = true;
                }
                break;
        }
    }

    public void autoUpdate() {
        switch (auto) {
            case DEF:
                break;

            case INIT_PARK:
                if (alliance == Alliance.BLUE) setAuto(AUTO.B_PARK);
                else if (alliance == Alliance.RED) setAuto(AUTO.R_PARK);
                else ;//выкидываем исключение
                break;

            case B_PARK:
                follower.followPath(paths.blueParking(follower.getPose()));
                break;

            case R_PARK:
                follower.followPath(paths.redParking(follower.getPose()));
                break; //аналогично

            case INIT_GOAL:
                if (alliance == Alliance.BLUE) setAuto(AUTO.B_GOAL);
                else if (alliance == Alliance.RED) setAuto(AUTO.R_GOAL);
                else ;//выкидываем исключение
                break;

            case R_GOAL:
                follower.followPath(paths.blueGoal(follower.getPose()));
                target = Shooter.VELOCITY_FOR_SHORT_THROW;
                setAutoState(AutoStates.INIT);
                break;

            case B_GOAL:
                follower.followPath(paths.redGoal(follower.getPose()));
                setAutoState(AutoStates.INIT);
                target = Shooter.VELOCITY_FOR_SHORT_THROW;
                break;

            case INIT_HUMAN:
                if (alliance == Alliance.BLUE) setAuto(AUTO.B_HUMAN);
                else if (alliance == Alliance.RED) setAuto(AUTO.R_HUMAN);
                else ;//выкидываем исключение
                break;

            case B_HUMAN:
                follower.followPath(paths.blueHuman(follower.getPose()));
                setAutoState(AutoStates.INIT);
                target = Shooter.VELOCITY_FOR_LONG_THROW;
                break;

            case R_HUMAN:
                follower.followPath(paths.redHuman(follower.getPose()));
                setAutoState(AutoStates.INIT);
                target = Shooter.VELOCITY_FOR_LONG_THROW;
                break;
        }
    }

    public void setAutoState(AutoStates state) {
        mode = MODE.AUTO;
        autoState = state;
        autoStTimer.reset();
    }

    public void setAuto(AUTO auto) {
        mode = MODE.AUTO;
        this.auto = auto;
        autoTimer.reset();
    }

    public void resetPose(Alliance alliance) {
        isPoseReset = true;          //tt.turnByTarget(0); //пересмотреть
        follower.setPose(new Pose(11, 7, 0));
        as.setAlliance(alliance);
    }

    public void driverUpdate(GamepadManager g1, GamepadManager g2) {
        g1.update();
        g2.update();

        /// =====================AUTO===============================///

        if (gamepad2.bWasPressed()) {
            setAuto(AUTO.INIT_PARK);
        }
        if (gamepad2.aWasPressed()) { //кнопки переписать
            setAuto(AUTO.INIT_GOAL);
        }
        if (gamepad2.xWasPressed()) {
            setAuto(AUTO.INIT_HUMAN);
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

        if (gamepad2.yWasPressed()) (sh.bonusShortVelocity) += 0.75;
        if (gamepad2.xWasPressed()) (sh.bonusShortVelocity) -= 0.75;
        if (gamepad2.bWasPressed()) (sh.bonusLongVelocity) += 0.75;
        if (gamepad2.aWasPressed()) (sh.bonusLongVelocity) -= 0.75;


//                if (g1.Y.isPressed() && g1.Y.getToggleState()) {
//                    sh.setLongThrowMode();
//                } else if (g1.X.isPressed() && g1.X.getToggleState()) {
//                    sh.setShortThrowMode();
//                } else if ((g1.Y.isPressed() && !g1.Y.getToggleState()) || (g1.X.isPressed() && !g1.X.getToggleState())) {
//                    sh.shootStop();
//                }
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

        /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

        if (g1.dpadRight.isPressed() && g1.dpadRight.getToggleState()) {
            attentionControl = true;
            sh.setManual(true);
            tt.turnByTarget(0);
        } else if (g1.dpadRight.isPressed() && !g1.dpadRight.getToggleState()) {
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

        if (g1.dpadUp.isPressed() && mode == MODE.AUTO) mode = MODE.DRIVER;
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}