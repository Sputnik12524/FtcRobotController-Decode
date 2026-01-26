package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleOpPP", group = "tele")
public class TeleOpPedro extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        // AUTO PARKING

        if (gamepad2.a) {
            follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY()));
            PathChain pathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(),
                                    new Pose(38.669, 33.4497)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(follower.getPose().getHeading()), Math.toRadians(90))
                    .build();
            follower.followPath(pathSecondScoring, true);
        }
        if (gamepad2.b) {
            follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY()));
            PathChain pathSecondScoring = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(),
                                    new Pose(105.568, 33.4497)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(follower.getPose().getHeading()), Math.toRadians(90))
                    .build();
            follower.followPath(pathSecondScoring, true);
        }


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}