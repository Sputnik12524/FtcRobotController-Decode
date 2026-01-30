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
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

import java.util.HashMap;
/*
@TeleOp(name = "TeleOpRR V2", group = "0")
@Config
public class TeleOpRoadRunnerV2 extends LinearOpMode {
    enum S {EMPTY_CHECK, INIT, SHOOT}
    S state = S.EMPTY_CHECK;
    GamepadManager g1;
    GamepadManager g2;
    Shooter sh;
    Transfer tr;
    Intake in;
    // Limelight ll;
    ElapsedTime timer;


    /// Intake
    boolean isRotateIn = false;
    boolean isShootingShort = false;
    boolean isShootingLong = false;
    boolean isRotateOut = false;

    /// Shooter
    boolean canShoot = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //  ll = new Limelight(this);
        timer = new ElapsedTime();
        sh = new Shooter(this);
        in = new Intake(this);
        tr = new Transfer(this);
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
            shoot();

            if (g2.A.isPressed()) {
                canShoot = true;
            }

            /// DRIVETRAIN
            if (g1.rightBumper.isHeld()) {
                dt.turnRightSlowMode();
            } else if (g1.leftBumper.isHeld()) {
                dt.turnLeftSlowMode();
            } else {
                dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }

            /// INTAKE
            if (g1.A.isPressed()) {
                if (g1.A.getToggleState()) {
                    in.transferSetPower(Intake.TRANSFER_POWER);
                    in.rotateIn();
                    isRotateIn = true;
                    isRotateOut = false;
                } else {
                    in.rotateStop();
                    in.transferSetPower(0);
                    isRotateIn = false;
                }
            }
            if (g1.B.isPressed()) {
                if (g1.B.getToggleState()) {
                    in.rotateOut();
                    isRotateOut = true;
                    isRotateIn = false;
                } else {
                    in.rotateStop();
                    isRotateOut = false;
                }
            }
            t.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            t.addData("Заброшенных артефактов", sh.artifacts);
            t.update();
        }

        if(g1.dpadUp.isHeldFor(2500)){
            while(opModeIsActive()){
                g1.update();
                g2.update();

                ///ЗАСЛОНКА
                if (g1.dpadUp.isPressed()) {
                    sh.openTunnel();
                } else if (g1.dpadDown.isPressed()) {
                    sh.closeTunnel();
                }

                /// SHOOTER
                if (g1.Y.isPressed() && !isShootingLong && g1.Y.getToggleState()) {
                    sh.setVelocityTarget(VELOCITY_FOR_LONG_THROW);
                    sh.setLongThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = true;
                    isShootingShort = false;
                } else if (g1.X.isPressed() && !isShootingShort && g1.X.getToggleState()) {
                    sh.setVelocityTarget(VELOCITY_FOR_SHORT_THROW);
                    sh.setShortThrowMode();
                    sh.shootByVelocity();
                    isShootingLong = false;
                    isShootingShort = true;
                } else if ((g1.X.isPressed() && !isShootingShort && g1.X.getToggleState()) || (g1.Y.isPressed() && !isShootingLong && g1.Y.getToggleState())) {
                    sh.closeTunnel();
                    sh.shootStop();
                    isShootingLong = false;
                    isShootingShort = false;
                }

                /// INTAKE
                if (g1.A.isPressed()) {
                    if (g1.A.getToggleState()) {
                        in.transferSetPower(Intake.TRANSFER_POWER);
                        in.rotateIn();
                        isRotateIn = true;
                        isRotateOut = false;
                    } else {
                        in.rotateStop();
                        in.transferSetPower(0);
                        isRotateIn = false;
                    }
                }
                if (g1.B.isPressed()) {
                    if (g1.B.getToggleState()) {
                        in.rotateOut();
                        isRotateOut = true;
                        isRotateIn = false;
                    } else {
                        in.rotateStop();
                        isRotateOut = false;
                    }
                }

                t.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
                t.addData("Заброшенных артефактов", sh.artifacts);
                t.update();

            }
        }
    }

    void shoot() {
        switch (state) {
            case INIT:
                sh.closeTunnel();
                if (tr.isEmpty() && !sh.shootingAllowed()) canShoot = false;
                else transit(S.SHOOT);
            case SHOOT:
                if(canShoot) sh.openTunnel();
                sh.threeArtefactsShooting();
                if (sh.completeC) {
                    sh.completeC = false;
                    transit(S.INIT);
                }
        }
    }

    void transit(S st) {
        state = st;
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}*/