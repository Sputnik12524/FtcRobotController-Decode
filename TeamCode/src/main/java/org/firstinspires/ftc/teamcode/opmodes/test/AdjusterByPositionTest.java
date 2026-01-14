package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;

@TeleOp(name = "TEST Adjuster by Botpose", group = "test")

public class AdjusterByPositionTest extends LinearOpMode {

    boolean stateY1 = false;
    boolean isShooting = false;

    @Override
    public void runOpMode() {
        Shooter sh = new Shooter(this);
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);

        PoseStorage.currentPose = dt.getPoseEstimate();
        dt.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        while (opModeIsActive()) {
            dt.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_trigger - gamepad1.left_trigger));

            dt.update();
            Pose2d current = dt.getPoseEstimate();
            if (current.getX() <= 23) {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_SHORT_THROW);
                sh.setShortThrowMode();
            } else {
                sh.setVelocityTarget(Shooter.VELOCITY_FOR_LONG_THROW);
                sh.setLongThrowMode();
            }


            if (gamepad1.y && !stateY1 && !isShooting) {
                sh.shootByVelocity();
                isShooting = true;
            } else if (gamepad1.y && !stateY1 && isShooting) {
                sh.shootStop();
                isShooting = false;
            }
            stateY1 = gamepad1.y;



            telemetry.addData("Pose X", current.getX());
            telemetry.addData("Pose Y", current.getY());
            telemetry.addData("heading", current.getHeading());

            telemetry.addData("Shooter servo pos", sh.getAngleAdjusterPos());
            telemetry.addData("Shooter Velo rps", sh.getVelocityRPS());
            telemetry.update();
        }


    }
    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}