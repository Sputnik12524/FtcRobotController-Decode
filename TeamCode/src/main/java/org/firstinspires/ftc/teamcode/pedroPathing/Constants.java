package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(11.1)
            .forwardZeroPowerAcceleration(-38.378604479027615).lateralZeroPowerAcceleration(-70.80644621428662)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.02606,0.2))
            .headingPIDFCoefficients(new PIDFCoefficients(2.1,0,0.00001,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.4,0,0.01,0.6,0.01))
            .centripetalScaling(0.00005)
            ;
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
            .yVelocity(35.379986519104946)
            .xVelocity(64.2857167569573)
            .useBrakeModeInTeleOp(true);


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0019857219417627296)
            .strafeTicksToInches(-0.001955627238534318)
            .turnTicksToInches(0.0019039361103468672)
            .leftPodY(3.4252)
            .rightPodY(-2.91339)
            .strafePodX(6.29921)
            .leftEncoder_HardwareMapName("leftBack")
            .rightEncoder_HardwareMapName("rightBack")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints).mecanumDrivetrain(driveConstants).threeWheelLocalizer(localizerConstants)
                .build();
    }
}
