package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.core.util.Supplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
@TeleOp(name = "Red TeleOpPP", group = "tele")
public class TeleOpPedro extends LinearOpMode {
    Shooter sh;
    Intake in;
    Limelight ll;
    ElapsedTime timer;
    Transfer tr;
    Follower follower;
    private Pose currentPose;

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
    private boolean automatedDrive;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        timer = new ElapsedTime();
        sh = new Shooter(this);
        in = new Intake(this);
        tr = new Transfer(this);
        Turret tt = new Turret(this);

        //currentPose = follower.getPose();
        isShootingLong = false;
        isShootingShort = false;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);


        //Lazy Curve Generation
        Supplier<PathChain> autoParkingRed = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.669, 33.4497))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        Supplier<PathChain> autoParkingBlue = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105.568, 33.4497))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        sh.closeTunnel();

        tt.turretRegulator.start();
        follower.update();


        waitForStart();
        follower.startTeleopDrive();
        while(opModeIsActive()){
            follower.update();
            telemetryM.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    true // Robot Centric
            );

            // AUTO PARKING
            if(gamepad2.dpadUpWasPressed() && !attentionControl){
                follower.followPath(autoParkingRed.get());
                automatedDrive = true;
            }
            if(gamepad1.dpadDownWasPressed() && !attentionControl){
                follower.followPath(autoParkingBlue.get());
                automatedDrive = true;
            }

            if (automatedDrive && (gamepad1.dpadLeftWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

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
            if (!attentionControl) {
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
                    tt.turnByTarget(0);
                } else if (gamepad2.a) {
                    tt.turnByTarget(180);
                } else if (gamepad2.x) {
                    tt.turnByTarget(115);
                } else if (gamepad2.b) {
                    tt.turnByTarget(62);
                }
            }
            telemetry.addData("ЭКСТРЕННОЕ УПРАВЛЕНИЕ:", attentionControl);
            telemetry.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            dashtele.addData("TARGET", tt.target);
            dashtele.addData("Current Pos", tt.getCurrentPosOfTurret());
            dashtele.addData("error", tt.error);
//            dashtele.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
//            dashtele.addData("Заброшенных артефактов", sh.artifacts);
//            dashtele.addData("Робот пустой?  ", tr.isEmpty());
//            dashtele.addData("Potuzhnaya stenka", sh.angleAdjuster.getPosition());
//            dashtele.addData("isSpinUp", sh.isSpinUp());
//            dashtele.addData("isDetected", sh.detected);

            dashtele.addData("Complete", sh.complete);
            dashtele.update();
            telemetry.update();
        }
        tt.turretRegulator.interrupt();

    }
}