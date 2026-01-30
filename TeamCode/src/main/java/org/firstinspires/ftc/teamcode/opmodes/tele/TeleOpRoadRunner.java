package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOpRR", group = "0")
@Config
public class TeleOpRoadRunner extends LinearOpMode {
    Shooter sh;
    Intake in;
    Limelight ll;
    ElapsedTime timer;
    Transfer tr;
    Follower follower;
    private Pose currentPose;

    public static double TURRET_ZERO = 0;
    public static double TURRET_MAX = 180;
    public static double TURRET_BLUE = 25;
    public static double TURRET_RED = -22;

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
    boolean stateRSB2 = false;
    private PathChain PathSecondScoring;
    public boolean weCanShoot = false;
    boolean detect = false;
    boolean attentionControl = false;


    @Override
    public void runOpMode() throws InterruptedException {


        // ll = new Limelight(this);
        follower = Constants.createFollower(hardwareMap);
        timer = new ElapsedTime();
        sh = new Shooter(this);
        in = new Intake(this);
        tr = new Transfer(this);
        Turret tt = new Turret(this);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        //currentPose = follower.getPose();
        isShootingLong = false;
        isShootingShort = false;
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);
        sh.closeTunnel();

        tt.turretRegulator.start();
        follower.update();


        waitForStart();

        while (opModeIsActive()) {

            //------------------------------------- DRIVETRAIN
            follower.update();

            if (gamepad1.right_bumper) {
                dt.turnRightSlowMode();
            } else if (gamepad1.left_bumper) {
                dt.turnLeftSlowMode();
            } else {
                dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }


            //-------------------------------------- INTAKE

            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                in.transferSetPower(Intake.TRANSFER_POWER);
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                in.transferSetPower(0);
                isRotateIn = false;
            }
            if (gamepad1.b && !isRotateOut && !stateB1) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
            } else if (gamepad1.b && isRotateOut && !stateB1) {
                in.rotateStop();
                isRotateOut = false;
            }
            stateA1 = gamepad1.a;
            stateB1 = gamepad1.b;

            //------------------------------------ SHOOTER
            if (attentionControl) {
                sh.threeArtefactsShooting();
            }

            if (gamepad1.y && !isShootingLong && !stateY1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShootingLong = true;
                isShootingShort = false;
            } else if (gamepad1.x && !isShootingShort && !stateX1) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.setShortThrowMode();
                sh.shootByVelocity();
                isShootingLong = false;
                isShootingShort = true;
            } else if ((gamepad1.y && !stateY1 && isShootingLong) || (gamepad1.x && !stateX1 && isShootingShort)) {
                sh.closeTunnel();
                sh.shootStop();
                isShootingLong = false;
                isShootingShort = false;
            }
            stateY1 = gamepad1.y;
            stateX1 = gamepad1.x;

            if (gamepad1.dpad_up) {
                sh.openTunnel();
            } else if (gamepad1.dpad_down) {
                sh.closeTunnel();
            }

            //---------------------------------------- TURRET

            if (!attentionControl) {
                tt.continuousTurnToGate(Alliance.RED, follower.getPose().getX(),
                        follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
            }

            /// -------------------------------------- ЭКСТРЕННОЕ УПРАВЛЕНИЕ

            if (gamepad2.right_stick_button) {
                attentionControl = true;
            } else if (gamepad2.left_stick_button) {
                attentionControl = false;
            }

            if (attentionControl) {
                if (gamepad2.y) {
                    tt.turnByTarget(TURRET_ZERO);
                } else if (gamepad2.a) {
                    tt.turnByTarget(TURRET_MAX);
                } else if (gamepad2.x) {
                    tt.turnByTarget(TURRET_BLUE);
                } else if (gamepad2.b) {
                    tt.turnByTarget(TURRET_RED);
                }
            }






            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            dashtele.addData("TARGET", tt.target);
            dashtele.addData("Current Pos", tt.getCurrentPosOfTurret());
            dashtele.addData("error", tt.error);
            dashtele.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            dashtele.addData("Заброшенных артефактов", sh.artifacts);
//            dashtele.addData("Робот пустой?  ", tr.isEmpty());
            dashtele.addData("aNGLE aDJUSTER (potuzhni)", sh.angleAdjuster.getPosition());
            dashtele.addData("isSpinUp", sh.isSpinUp());
//            dashtele.addData("isDetected", sh.detected);

            dashtele.addData("Complete", sh.complete);
            dashtele.update();
            telemetry.update();
        }
        tt.turretRegulator.interrupt();

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}