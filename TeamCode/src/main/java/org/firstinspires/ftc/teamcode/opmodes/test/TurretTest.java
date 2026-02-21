package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;

@Config
@TeleOp(name = "TEST Turret", group = "1")
public class TurretTest extends LinearOpMode {

    Follower follower;
    AutoSniper as;
    Turret tt;
    Shooter sh;


    @Override
    public void runOpMode() {
        tt = new Turret(this);
        as = new AutoSniper(tt, sh);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();

        tt.turretRegulator.start();

        as.setAlliance(Alliance.RED);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            as.continuousTurnTurretToGate( follower.getPose().getX(), follower.getPose().getY() ,Math.toDegrees(follower.getHeading()));
            telemetry.addData("target", tt.target);
            telemetry.addData("error", tt.error);
            telemetry.addData("target FROM AutoSniper", as.target);
            telemetry.addData("angleOfTurret (отн. поля)", as.angleOfTurret);
            telemetry.addData("x:", follower.getPose().getX());
            telemetry.addData("y:", follower.getPose().getY());
            telemetry.addData("head", follower.getPose().getHeading());
            telemetry.update();

        }

        tt.turretRegulator.interrupt();

    }
}
