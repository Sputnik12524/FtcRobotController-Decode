package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.DriveTrain;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="TeleOpPedro", group="tele")
public class TeleOpPedro extends LinearOpMode {
    DriveTrain dt;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode(){
        dt = new DriveTrain(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();
        follower.startTeleopDrive();
        if(!isStopRequested()) return;
        follower.update();
        telemetryM.update();

        //Make the last parameter false for field-centric
        //In case the drivers want to use a "slowMode" you can scale the vectors

        //This is the normal version to use in the TeleOp
        dt.setPower(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.left_trigger-gamepad1.right_trigger);
        telemetryM.debug("position", follower.getPose());
    }
}