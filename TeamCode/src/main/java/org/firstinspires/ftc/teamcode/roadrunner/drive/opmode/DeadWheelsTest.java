package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp(name="TEST Dead Wheels", group="test")
@Disabled
public class DeadWheelsTest extends LinearOpMode {
    public Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack")); //2
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack")); //0
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); //1
        DriveTrainMecanum drive = new DriveTrainMecanum(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("X Left - ", leftEncoder.getCurrentPosition());
            telemetry.addData("Y - ", frontEncoder.getCurrentPosition());
            telemetry.addData("X Right - ", rightEncoder.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Current pose X = ", drive.getPoseEstimate().getX());
            telemetry.addData("Current pose Y = ", drive.getPoseEstimate().getY());
            telemetry.addData("Current heading =", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
