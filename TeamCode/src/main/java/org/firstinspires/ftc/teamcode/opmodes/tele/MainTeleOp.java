package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Parking;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
@TeleOp(name = "MainTeleop", group = "Robot")

public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain dt = new DriveTrain();
        Intake in = new Intake();
        Shooter st = new Shooter();
        Parking pk = new Parking();
        Sorting sr = new Sorting();

        waitForStart();
        while (opModeIsActive()) {
        }
    }
}
