package org.firstinspires.ftc.teamcode.opmodes.test.camera;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "TEST camera", group = "1")
public class CameraTurretTest extends LinearOpMode {

    Follower follower;
    Limelight ll;
    Turret tt;
    Shooter sh;


    @Override
    public void runOpMode() {
        ll = new Limelight(this);
        tt = new Turret(this, ll);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        tt.turretRegulator.start();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            tt.turnByTarget(tt.stabilizeTargetByCamera());
            telemetry.addData("target", tt.target);
            telemetry.addData("error", tt.error);
            telemetry.update();

        }

        tt.turretRegulator.interrupt();

    }
}
