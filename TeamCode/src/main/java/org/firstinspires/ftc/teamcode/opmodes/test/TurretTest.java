package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public static double TARGET = 0;

    @Override
    public void runOpMode() {
        as = new AutoSniper();
        tt = new Turret(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, 0));
        follower.update();

        as.setAlliance(Alliance.RED);

        waitForStart();

        while (opModeIsActive()) {

            as.continuousTurnTurretToGate( follower.getPose().getX(), follower.getPose().getY() ,follower.getHeading());
            telemetry.addData("target", tt.target);
            telemetry.update();

        }

    }
}
