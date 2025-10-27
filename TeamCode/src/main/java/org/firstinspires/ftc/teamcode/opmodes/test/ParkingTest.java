package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Sorting;

@TeleOp(name = "TestSort", group = "Robot")
public class ParkingTest extends LinearOpMode {
    //private final DcMotor shooter;

    @Override
    public void runOpMode() {
        //DcMotor drum = this.hardwareMap.get(DcMotor.class, "drum");
        Sorting sorting = new Sorting(this);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //drum.setPower(gamepad1.left_stick_y);
           // shooter.setPower(gamepad1.right_stick_y);
            sorting.drumTele(gamepad1.left_stick_y/10);
            sorting.shootTele(gamepad1.right_stick_y);

            if(gamepad1.a){
                sorting.drumTele(0.2);
            }
            if(gamepad1.b){
                sorting.drumTele(0.4);
            }
            if(gamepad1.x){
                sorting.drumTele(0.6);
            }
            if(gamepad1.y){
                sorting.drumTele(0.8);
            }
            while (gamepad1.dpad_up){
                sorting.drumTele(0.1);
            }
            if (gamepad1.dpad_right){
                sorting.drumTele(0.5);
            }
        }
    }



}
