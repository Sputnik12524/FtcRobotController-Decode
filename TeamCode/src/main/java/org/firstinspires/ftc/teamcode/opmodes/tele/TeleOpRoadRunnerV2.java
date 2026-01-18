package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.firstinspires.ftc.teamcode.modules.Shooter.VELOCITY_FOR_LONG_THROW;
import static org.firstinspires.ftc.teamcode.modules.Shooter.VELOCITY_FOR_SHORT_THROW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

@TeleOp(name = "TeleOpRR V2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    GamepadManager g1;
    GamepadManager g2;

    enum Calc {DEFAULT, FIELD_POSE, INIT, LONG, SHORT}

    Calc position = Calc.DEFAULT;

    Shooter sh;
    Intake in;
    Limelight ll;
    ElapsedTime timer;


    /// Intake
    boolean isRotateIn = false;
    boolean isShootingShort = false;
    boolean isShootingLong = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean stateRB1 = false;
    boolean isShooting = false;
    boolean longSh = false;
    boolean shortSh = false;
    int artefactsIn = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        ll = new Limelight(this);
        timer = new ElapsedTime();
        sh = new Shooter(this);
        in = new Intake(this);
        isShootingLong = false;
        isShootingShort = false;
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        g1 = new GamepadManager(gamepad1);
        g2 = new GamepadManager(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);
        sh.closeTunnel();


        waitForStart();

        while (opModeIsActive()) {
            g1.update();
            g2.update();
            switch (position) {

                case DEFAULT:
                    if (isShooting && timer.milliseconds() > sh.timers) transit(Calc.FIELD_POSE);
                    break;

                case FIELD_POSE:
                    //if можно ли стрелять???? transit(Calc.INIT);
                    break;

                case INIT:

                    if (longSh && sh.isBack(VELOCITY_FOR_LONG_THROW)) transit(Calc.LONG);
                    if (shortSh && sh.isBack(VELOCITY_FOR_SHORT_THROW)) transit(Calc.SHORT);
                    break;

                case LONG:
                    sh.openTunnel();
                    sh.updateCalculator(VELOCITY_FOR_LONG_THROW);
                    transit(Calc.DEFAULT);
                    break;

                case SHORT:
                    sh.openTunnel();
                    sh.updateCalculator(VELOCITY_FOR_SHORT_THROW);
                    transit(Calc.DEFAULT);
                    break;
            }
            longSh = false;
            shortSh = false;

            if (artefactsIn != 0 && !sh.canShoot) {
                sh.closeTunnel();
                artefactsIn--;
            }

            // DRIVETRAIN
            if (g1.rightBumper.isHeld()) {
                dt.turnRightSlowMode();
            } else if (g1.leftBumper.isHeld()) {
                dt.turnLeftSlowMode();
            } else {
                dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }

            // INTAKE
            if (g1.A.isPressed()) {
                if (g1.A.getToggleState()) {
                    in.rotateIn();
                } else {
                    in.rotateStop();
                }
            }
            if (g1.B.isPressed()) {
                if (g1.B.getToggleState()) {
                    in.rotateOut();
                } else  {
                    in.rotateStop();
                }
            }

            // SHOOTER
            if (g1.rightBumper.isPressed() && g1.rightBumper.getToggleState()) {
                sh.needShootPortion();
            }


            if (g1.Y.isPressed() && !isShootingLong && g1.Y.getToggleState()) {
                sh.setVelocityTarget(VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShootingLong = true;
                isShootingShort = false;
            } else if (g1.X.isPressed()&& !isShootingShort && g1.X.getToggleState()) {
                sh.setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
                sh.setShortThrowMode();
                sh.shootByVelocity();
                isShootingLong = false;
                isShootingShort = true;
            } else if ((g1.X.isPressed()&& !isShootingShort && g1.X.getToggleState()) || (g1.Y.isPressed() && !isShootingLong && g1.Y.getToggleState())){
                sh.closeTunnel();
                sh.shootStop();
                isShootingLong = false;
                isShootingShort = false;
            }
            stateY1 = gamepad1.y;
            stateX1 = gamepad1.x;

            if (g1.dpadUp.isPressed()) {
                sh.openTunnel();
            } else if (g1.dpadDown.isPressed()) {
                sh.closeTunnel();
            }

            t.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            t.addData("Заброшенных артефактов", sh.artifacts);
            t.addData("Робот: ", sh.isEmpty());
            t.update();


        }
    }

    public void transit(Calc state) {
        position = state;
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}