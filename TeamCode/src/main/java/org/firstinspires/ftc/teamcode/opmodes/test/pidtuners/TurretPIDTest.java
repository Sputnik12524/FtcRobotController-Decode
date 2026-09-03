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
        AutoSniper as = new AutoSniper(tt, new Shooter(this), ll);
        DriveTrain dt = new DriveTrain(this);
        boolean magnetic = false;

        t = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        fl.setStartingPose(new Pose(72,72,0));
        fl.update();

        kP = Turret.kPL;
        kI = Turret.kIL;

        ll.startOrStopLL(false);
        as.setAlliance(Alliance.RED);
        tt.turretRegulator.start();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();


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


            if (tt.isMagneting() && !magnetic) {
                magnetic = true;

            }
            if (gamepad2.aWasPressed()) magnetic = false;

            if (turretState) {

                t.addLine("TURRET TELEMETRY:");
                t.addData("target", tt.target);
                t.addData("current", tt.getCurrentPosOfTurret());
                t.addData("ll_weight", tt.ll_weight);
                t.addData("aim method", tt.getAimMethod());
                t.addData("tx", ll.getGoalTag()[1]);
            } else {
                t.addLine("TURRET is stopped (put X)");
                tt.turnByTarget(0);
                turretState = false;
            }




            dt.setMotorsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

            t.addData("Magnetic state", tt.isMagneting());
            t.addData("Magnetic state", magnetic);

            t.update();

        }
        ll.startOrStopLL(true);
        tt.turretRegulator.interrupt();
    }
}
