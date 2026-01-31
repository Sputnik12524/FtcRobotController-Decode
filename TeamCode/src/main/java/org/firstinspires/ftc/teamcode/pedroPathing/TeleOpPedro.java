package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.core.util.Supplier;

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
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "TeleOpPP", group = "tele")
public class TeleOpPedro extends LinearOpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private ElapsedTime t;
    private ElapsedTime tS;

    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        t = new ElapsedTime();


        //Lazy Curve Generation
        Supplier<PathChain> autoParkingRed = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.669, 33.4497))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        Supplier<PathChain> autoParkingBlue = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105.568, 33.4497))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        waitForStart();
        follower.startTeleopDrive();
        while(opModeIsActive()){
            follower.update();
            telemetryM.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    true // Robot Centric
            );

            // AUTO PARKING
            if(gamepad1.aWasPressed()){
                follower.followPath(autoParkingRed.get());
                automatedDrive = true;
            }
            if(gamepad1.bWasPressed()){
                follower.followPath(autoParkingBlue.get());
                automatedDrive = true;
            }

            if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }


            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("automatedDrive", automatedDrive);
        }
    }
}