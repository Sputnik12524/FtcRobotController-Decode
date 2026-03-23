package org.firstinspires.ftc.teamcode.opmodes.test.pidtuners;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;


@TeleOp
@Config
public class TurretPIDTest extends LinearOpMode {
    public static double kP, kI, kD;

    @Override
    public void runOpMode() {
        Follower fl = Constants.createFollower(hardwareMap);
        Limelight ll = new Limelight(this);
        Turret tt = new Turret(this, ll);
        AutoSniper as = new AutoSniper(tt, new Shooter(this));

        fl.setStartingPose(new Pose(72,72,0));

        as.setAlliance(Alliance.RED);
        tt.turretRegulator.start();

        waitForStart();

        while (opModeIsActive()) {

            tt.tuneTurretPID(kP, kI, kD);
            as.continuousTurnTurretToGate(fl.getPose().getX(), fl.getPose().getY(), fl.getHeading());

            telemetry.addData("Follower x", fl.getPose().getX());
            telemetry.addData("Follower y", fl.getPose().getY());
            telemetry.addData("Follower heading", fl.getPose().getHeading());
            telemetry.update();

        }
        tt.turretRegulator.interrupt();
    }
}
