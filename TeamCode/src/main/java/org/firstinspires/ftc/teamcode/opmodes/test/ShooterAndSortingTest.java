package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Sorting sr;
    Limelight ll;

    @Override
    public void runOpMode() {
        sh = new Shooter(this);
        ll = new Limelight(this);
        in = new Intake(this);
        sr = new Sorting(this);

        sr.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}


