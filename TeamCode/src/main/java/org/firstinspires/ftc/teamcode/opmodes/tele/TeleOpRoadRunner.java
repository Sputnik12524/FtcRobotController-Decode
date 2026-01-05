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

@TeleOp(name = "TeleOpRR", group = "0")
@Config
public class TeleOpRoadRunner extends LinearOpMode {
    enum Calc {DEFAULT, INIT, LONG, SHORT}
    Calc position = Calc.DEFAULT;
    Shooter sh;
    Intake in;
    Limelight ll;
    ElapsedTime timer;


    /// Intake
    boolean isRotateIn = false;
    boolean isShooting = false;
    boolean longSh = false;
    boolean shortSh = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;
    int artefactsIn = 0; //How many artefacts in robot (calculate by Intake velocity)

    /// Shooter
    boolean stateY1 = false;
    boolean stateX1 = false;
    boolean stateRB1 = false;


    @Override
    public void runOpMode() throws InterruptedException {

        ll = new Limelight(this);
        timer = new ElapsedTime();
        sh = new Shooter(this);
        in = new Intake(this);
        isShooting = false;
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        sh.portion.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);


        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            switch (position) {
                case DEFAULT:
                    if (isShooting && timer.milliseconds() > sh.timers) transit(Calc.INIT);
                case INIT:
                    if (longSh && sh.isBack(VELOCITY_FOR_LONG_THROW)) transit(Calc.LONG);
                    if (shortSh && sh.isBack(VELOCITY_FOR_SHORT_THROW)) transit(Calc.SHORT);
                case LONG:
                    sh.openCover();
                    sh.updateCalculator(VELOCITY_FOR_LONG_THROW);
                    transit(Calc.DEFAULT);
                case SHORT:
                    sh.openCover();
                    sh.updateCalculator(VELOCITY_FOR_SHORT_THROW);
                    transit(Calc.DEFAULT);
            }
            longSh = false;
            shortSh = false;

            if(artefactsIn != 0 && !sh.canShoot){
                sh.closeCover();
                artefactsIn --;
            }

            // DRIVETRAIN
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

            // INTAKE
            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
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

            // SHOOTING
            if (gamepad1.right_bumper && !stateRB1) {
                sh.needShootPortion();
            }
            stateRB1 = gamepad1.right_bumper;

            if (gamepad1.y && !isShooting && !stateY1) {
                sh.openCover();
                sh.setVelocityTarget(VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShooting = true;
                longSh = true;
                artefactsIn = 3;
            } else if (gamepad1.x && !isShooting && !stateX1) {
                sh.openCover();
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.setShortThrowMode();
                sh.shootByVelocity();
                isShooting = true;
                shortSh = true;
            } else if (((gamepad1.y && !stateY1) || (gamepad1.x && !stateX1)) && isShooting) {
                sh.closeCover();
                sh.shootStop();
                isShooting = false;
            }
            stateY1 = gamepad1.y;
            stateX1 = gamepad1.x;

            if (gamepad1.dpad_up) {
                sh.closeCover();
            } else if (gamepad1.dpad_down) {
                sh.openCover();
            }

            t.addData("Velocity shooter", sh.shooterUpper.getVelocity() / 28);
            t.addData("Заброшенных артефактов", sh.artifacts);
            t.addData("Is shooting?", isShooting);
            t.update();
        }
        sh.portion.interrupt();
    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

    public void transit(Calc state) {
        position = state;
    }
}