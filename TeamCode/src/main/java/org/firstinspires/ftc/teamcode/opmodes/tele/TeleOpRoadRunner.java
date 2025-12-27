package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.firstinspires.ftc.teamcode.opmodes.test.pidtuners.VeloPIDTuner.MOTOR_VELO_PID_SHOOTERS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name = "TeleOpRR Shooter - Velocity", group = "0")
@Config
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter sh;
    Intake in;
    Limelight ll;
    ElapsedTime timer;


    /// Intake
    boolean isRotateIn = false;
    boolean isShooting = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

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
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
                sh.shootByVelocity();
                isShooting = true;
            } else if (gamepad1.x && !isShooting && !stateX1) {
                sh.openCover();
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.setShortThrowMode();
                sh.shootByVelocity();
                isShooting = true;
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
}