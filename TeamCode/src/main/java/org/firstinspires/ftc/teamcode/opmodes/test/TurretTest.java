package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Config
@TeleOp(name = "TEST Turret", group = "1")
public class TurretTest extends LinearOpMode {

    Turret tr;
    Follower follower;

    public static double TARGET = 0;

    @Override
    public void runOpMode() {
        tr = new Turret(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();



        tr.turretRegulator.start();
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
//            if (gamepad1.right_bumper) {
//                tr.turnRightByTarget();
//            } else if (gamepad1.left_bumper) {
//                tr.turnLeftByTarget();
//            }

            tr.continuousTurnToGate(Alliance.RED, follower.getPose().getX(),
                    follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("Magneting?", tr.isMagneting());
            telemetry.addData("speed", tr.turret.getVelocity());
            telemetry.addData("target", tr.target);
            telemetry.addData("error", tr.error);
            telemetry.addData("Pos", tr.getCurrentPosOfTurret());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Head", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
        tr.turretRegulator.interrupt();
    }
}
