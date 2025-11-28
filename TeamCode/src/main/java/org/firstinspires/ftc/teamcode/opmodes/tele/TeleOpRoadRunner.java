package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name = "TeleOpRR", group = "0")
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter sh;
    Intake in;
    Sorting st;
    Limelight ll;
    ElapsedTime timer;
    /// Intake
    boolean isRotateIn = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

    /// Shooter
    boolean isShooting = false;
    boolean stateA2 = false;

    /// Sorting
    boolean isOpen = false;
    boolean stateY2 = false;
    boolean stateY1 = false;
    boolean stateB2 = false;
    boolean stateDown1 = false;
    boolean stateRight1 = false;
    boolean stateUp1 = false;
    boolean stateLeft1 = false;
    boolean stateLeft2 = false;
    boolean stateDown2 = false;
    boolean stateRight2 = false;
    boolean stateUp2 = false;

    public static double POWER_LOWEST = 0.1;

    public static double POWER_HIGHEST = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        ll = new Limelight(this);
        timer = new ElapsedTime();
        st = new Sorting(this);
        sh = new Shooter(this);
        in = new Intake(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        while (opModeIsActive()) {
            // DRIVETRAIN
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);


            // INTAKE and SORTING
            if (gamepad1.a && !isRotateIn && !stateA1) {
                st.wallForIntaking();
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
            }

            // SHOOTER and SORTING стрельба с дальней
            if (gamepad2.a && !isShooting && !stateA2) {
                st.wallForShooting();
                sh.shootByPower(-POWER_HIGHEST);
                sh.shooter2.setPower(POWER_HIGHEST);
                isShooting = true;
            } else if (gamepad2.a && isShooting && !stateA2) {
                sh.shootStop();
                sh.shooter2.setPower(0);
                isShooting = false;
            }

            //стрельба с ближней 0.76
            if (gamepad2.y && !isShooting && !stateY2) {
                st.wallForShooting();
                sh.shootByPower(-POWER_LOWEST);
                sh.shooter2.setPower(POWER_HIGHEST);
                isShooting = true;
            } else if (gamepad2.y && isShooting && !stateY2) {
                sh.shootStop();
                sh.shooter2.setPower(0);
                isShooting = false;
            }

            // увеличение мощности при стрельбе
            if (gamepad2.dpad_up && isShooting && !stateUp2) {
                st.wallForShooting();
                POWER_LOWEST += 0.05;
                sh.shootByPower(-POWER_LOWEST);
                isShooting = true;
            }

            // уменьшение мощности при стрельбе
            if (gamepad2.dpad_down && isShooting && !stateDown2) {
                st.wallForShooting();
                POWER_LOWEST -= 0.05;
                sh.shootByPower(-POWER_LOWEST);
                isShooting = true;
            }

            //IF TROUBLES
            if (gamepad1.b && !isRotateOut && !stateB1) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
            } else if (gamepad1.b && isRotateOut && !stateB1) {
                in.rotateStop();
                isRotateOut = false;
            }


            if (gamepad2.x) {
                st.drumTeleGo();
            } else {
                st.drumTeleStop();
            }

            if (gamepad2.dpad_left) {
                st.drumTeleGoRevers();
            } else {
                st.drumStop();
            }


            if (gamepad2.b && !isOpen && !stateB2) {
                st.horizontalWallOpen();
                isOpen = true;
            } else if (gamepad2.b && isOpen && !stateB2) {
                st.horizontalWallClose();
                isOpen = false;
            }

        stateA1 = gamepad1.a;
        stateB1 = gamepad1.b;
        stateA2 = gamepad2.a;
        stateB2 = gamepad2.b;
        stateY2 = gamepad2.y;
        stateY1 = gamepad1.y;
        stateDown2 = gamepad2.dpad_down;
        stateRight2 = gamepad2.dpad_right;
        stateUp2 = gamepad2.dpad_up;
        stateDown1 = gamepad1.dpad_down;
        stateRight1 = gamepad1.dpad_right;
        stateUp1 = gamepad1.dpad_up;
        stateLeft1 = gamepad1.dpad_left;
        stateLeft2 = gamepad2.dpad_left;
        telemetry.addData("Power shooter", sh.shooter.getPower());
        telemetry.update();
    }
}


public static class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}

}