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
    boolean isOpen = false;
    boolean stateY2 = false;
    boolean stateY1 = false;
    boolean stateB2 = false;
    boolean stateDown1 = false;
    boolean stateRight1 = false;
    boolean stateUp1 = false;
    boolean stateLeft1 = false;
    boolean stateDown2 = false;
    boolean stateRight2 = false;
    boolean stateUp2 = false;

    public static double POWER_LOWEST = 0.76;
    public static double POWER_MIDDLE = 0.8;
    public static double POWER_HIGHEST = 0.85;


    @Override
    public void runOpMode() throws InterruptedException {

        ll = new Limelight(this);
        timer = new ElapsedTime();
        sorting = new Sorting(this);
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
                sorting.wallForIntaking();
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
            }

             //SORTING
//            if (gamepad1.dpad_down && !stateDown1) {
//                sorting.drumMotor.setPower(0.2);
//                while(sorting.drumMotor.getCurrentPosition() % 120 == 0);
//                sorting.drumStop();
//                sorting.regulatorSorting.start();
//                sorting.sortingArtefacts(sorting.artefact_pos(sorting.getColor()), Sorting.Scan.LEFT);
//            }
//            if (gamepad1.dpad_right && !stateRight1) {
//                sorting.regulatorSorting.start();
//                sorting.sortingArtefacts(sorting.artefact_pos(sorting.getColor()), Sorting.Scan.BETWEEN);
//            }
//            if (gamepad1.dpad_up && !stateUp1) {
//                sorting.regulatorSorting.start();
//                sorting.sortingArtefacts(sorting.artefact_pos(sorting.getColor()), Sorting.Scan.RIGHT);
//            }
//            if (gamepad1.dpad_left && !stateLeft1) {
//                sorting.regulatorSorting.interrupt();
//            }

            // SHOOTER and SORTING с дальней
            if (gamepad2.a && !isShooting && !stateA2) {
                sorting.wallForShooting();
                st.shootByPower(1);
                isShooting = true;
            } else if (gamepad2.a && isShooting && !stateA2) {
                st.shootStop();
                isShooting = false;
            }

            //стрельба с ближней 0.76
            if (gamepad2.y && !isShooting && !stateY2) {
                sorting.wallForShooting();
                st.shootByPower(POWER_LOWEST);
                isShooting = true;
            } else if (gamepad2.y && isShooting && !stateY2) {
                st.shootStop();
                isShooting = false;
            }

            // увеличение мощности при стрельбе
            if (gamepad2.dpad_up && isShooting && !stateUp2) {
                sorting.wallForShooting();
                POWER_LOWEST += 0.05;
                isShooting = true;
            }

            // уменьшение мощности при стрельбе
            if (gamepad2.dpad_down && isShooting && !stateDown2) {
                sorting.wallForShooting();
                POWER_LOWEST -= 0.05;
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
                sorting.drumTeleGo();
            } else {
                sorting.drumTeleStop();
            }

            if (gamepad2.b && !isOpen && !stateB2) {
                sorting.horizontalWallOpen();
                isOpen = true;
            } else if (gamepad2.b && isOpen && !stateB2) {
                sorting.horizontalWallClose();
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
            stateDown1= gamepad1.dpad_down;
            stateRight1 = gamepad1.dpad_right;
            stateUp1 = gamepad1.dpad_up;
            stateLeft1 = gamepad1.dpad_left;
        }
    }


    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }

}