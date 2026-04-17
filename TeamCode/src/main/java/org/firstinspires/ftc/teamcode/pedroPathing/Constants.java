package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(15.5)
            .forwardZeroPowerAcceleration(-35.64039824453843)
            .lateralZeroPowerAcceleration(-51.308022973952156)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.015,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1.01,0,0.099,0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.00001,0.01,0.6))
            .centripetalScaling(0.00005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
            .rightRearMotorDirection(DcMotor.Direction.REVERSE)
            .yVelocity(28.99685885211614)

            .xVelocity(63.24041219395916)
            .useBrakeModeInTeleOp(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.4252)
            .strafePodX(-6.29921)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints).mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
