package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name = "TeleOpRR", group = "Tele")
public class TeleOpRoadRunner extends LinearOpMode {

    Shooter st;
    Intake in;
    Sorting sorting;
    ElapsedTime timer;
    /// Intake
    boolean isRotateIn = false;
    boolean isRotateOut = false;
    boolean stateA1 = false;
    boolean stateB1 = false;

    /// Shooter
    boolean isShooting = false;
    boolean stateA2 = false;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        sorting = new Sorting(this);
        st = new Shooter(this);
        in = new Intake(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        st.setShortThrow();

        waitForStart();

        while (opModeIsActive()) {
            // DRIVETRAIN
            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

            // INTAKE and SORTING
            if (gamepad1.a && !isRotateIn && !stateA1) {
                sorting.dwallOpen();
                sorting.hwallClose();
                in.rotateIn();
                isRotateIn = true;
                //isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
            }



//            if (gamepad1.b && !isRotateOut && !stateB1) {
//                in.rotateOut();
//                isRotateOut = true;
//                isRotateIn = false;
//            } else if (gamepad1.b && isRotateOut && !stateB1) {
//                in.rotateStop();
//                isRotateOut = false;
//            }
            stateA1 = gamepad1.a;
//            stateB1 = gamepad1.b;

            // SHOOTER and SORTING
            if (gamepad2.a && !isShooting && !stateA2) {
                sorting.dwallClose();
                sorting.hwallOpen();
                st.shoot();
                while(timer.milliseconds() < 500){}
                isShooting = true;
            } else if (gamepad2.a && isShooting && !stateA2) {
                st.shootStop();
                isShooting = false;
            }
            stateA2 = gamepad2.a;

            if (gamepad2.b) {
                sorting.drumTele();
            } else {
                sorting.drumStop();
            }

        }

    }

    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}
