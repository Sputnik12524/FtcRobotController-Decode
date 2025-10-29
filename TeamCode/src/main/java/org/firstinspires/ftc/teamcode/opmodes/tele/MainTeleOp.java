package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.drivetrainrr.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.modules.drivetrainrr.ThreeWheelLocalizer;

@TeleOp(name="TeleOp RoadRunner", group="Robot")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainMecanum dt = new DriveTrainMecanum(hardwareMap);
        ThreeWheelLocalizer loc = new ThreeWheelLocalizer(hardwareMap);

        waitForStart();

    }
}
