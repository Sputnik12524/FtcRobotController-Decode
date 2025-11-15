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

@TeleOp(name = "TeleOpRR", group = "Tele")
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter st;
    Intake in;
    Sorting sorting;
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
    boolean stateB2 = false;
    boolean stateY = false;


    @Override
    public void runOpMode() throws InterruptedException {

        ll = new Limelight(this);
        timer = new ElapsedTime();
        sorting = new Sorting(this, ll);
        st = new Shooter(this);
        in = new Intake(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);
        //sorting.regulatorSorting.start();

        waitForStart();

        while (opModeIsActive()) {
            // DRIVETRAIN
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);


            // INTAKE and SORTING
            if (gamepad1.a && !isRotateIn && !stateA1) {
                //sorting.intaker.start();
                sorting.wallForIntaking();
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                //sorting.intaker.interrupt();
                isRotateIn = false;
            }

            // SORTING
            //  if (gamepad2.b && !stateB2) {
            // sorting.sortingArtefacts(sorting.artefact_pos(sorting.getColor()), sorting.aprilTagToScan(sorting.scanId));
//            }

            // SHOOTER and SORTING с дальней
            if (gamepad2.a && !isShooting && !stateA2) {
                sorting.wallForShooting();
                st.shootByPower(1);
                isShooting = true;
                //sorting.shooter.start();
            } else if (gamepad2.a && isShooting && !stateA2) {
                //sorting.shooter.interrupt();
                st.shootStop();
                isShooting = false;
            }

            //стрельба с ближней
            if (gamepad2.y && !isShooting && !stateY) {
                sorting.wallForShooting();
                st.shootByPower(0.76);
                isShooting = true;
            } else if (gamepad2.y && isShooting && !stateY) {
                st.shootStop();
                isShooting = false;
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
                sorting.drumTeleGo();
            } else {
                sorting.drumTeleStop();
            }

            stateA1 = gamepad1.a;
            stateB1 = gamepad1.b;
            stateA2 = gamepad2.a;
            stateB2 = gamepad2.b;
            stateY = gamepad2.y;
        }
    }


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

}