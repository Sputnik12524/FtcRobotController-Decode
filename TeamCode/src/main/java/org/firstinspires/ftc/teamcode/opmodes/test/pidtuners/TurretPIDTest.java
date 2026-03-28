package org.firstinspires.ftc.teamcode.opmodes.test.pidtuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AimingMethod;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSniper;


@TeleOp
@Config
public class TurretPIDTest extends LinearOpMode {
    public static double kP, kI, kD, kPC, kDC;

    boolean xState = false;
    boolean turretState = false;
    MultipleTelemetry t;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        Follower fl = Constants.createFollower(hardwareMap);
        Limelight ll = new Limelight(this);
        Turret tt = new Turret(this, ll);
        AutoSniper as = new AutoSniper(tt, new Shooter(this));
        DriveTrain dt = new DriveTrain(this);
        t = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        fl.setStartingPose(new Pose(72,72,0));
        fl.update();

        ll.startOrStopLL(false);
        as.setAlliance(Alliance.BLUE);
        tt.turretRegulator.start();

        waitForStart();

        while (opModeIsActive()) {


            fl.update();

            tt.tuneTurretPID(kP, kI, kD, kPC, kDC);

            //---------------------------------------------- TURRET

            if (gamepad1.x && !xState && !turretState) {
                as.enableAutoTurretAiming(true);
                turretState = true;
            } else if (gamepad1.x && !xState && turretState) {
                as.enableAutoTurretAiming(false);
                turretState = false;
            }
            xState = gamepad1.x;

            if (turretState) {
                as.continuousTurnTurretToGate(
                        fl.getPose().getX(),
                        fl.getPose().getY(),
                        fl.getHeading()
                );
 
                t.addLine("TURRET TELEMETRY:");
                t.addData("target", tt.target);
                t.addData("current", tt.getCurrentPosOfTurret());
                t.addData("error", tt.error);
                t.addData("target FROM AutoSniper", as.target);
                t.addData("angleOfTurret (отн. поля)", as.angleOfTurret);
            } else {
                t.addLine("TURRET is stopped (put X)");
                tt.turnByTarget(0);
                turretState = false;
            }
//            if(ll.getGoalTag().get(0) == 20 || ll.getGoalTag().get(0) == 24) {
//                tt.setAimMethod(AimingMethod.CAMERA);
//
//            } else {
//                tt.setAimMethod(AimingMethod.LOCALIZATION);
//                as.continuousTurnTurretToGate(fl.getPose().getX(), fl.getPose().getY(), fl.getHeading());
//        }
//            if(gamepad1.a){
//                tt.turret.setPower(0.3);
//            } else if (gamepad1.b){
//                tt.turret.setPower(-0.3);
//            } else {
//                tt.turret.setPower(0);
//            }



            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);


            t.addData("Follower x", fl.getPose().getX());
            t.addData("Follower y", fl.getPose().getY());
            t.addData("Follower heading", fl.getPose().getHeading());
            t.addData("Tx", ll.getTagInfo().get(1) );
            t.addData("Aim method", tt.getAimMethod());
            t.update();

        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }
}
