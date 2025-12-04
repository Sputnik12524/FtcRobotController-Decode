package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Parking;

@Config
@TeleOp(name = "ParkingTest", group = "Test")
public class ParkingTest extends LinearOpMode {
    Parking park;
    boolean isParking = false;
    boolean stateA = false;
    boolean stateB = false;
    public static double POWER = 1;

    @Override
    public void runOpMode() {
        park = new Parking(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !isParking && !stateA) {
                park.parkingPower(POWER);
                isParking = true;
            } else if (gamepad1.a && isParking && !stateA) {
                park.parkingStop();
                isParking = false;
            }

            if (gamepad1.b && !isParking && !stateB) {
                park.parkingPower(-POWER);
                isParking = true;
            } else if (gamepad1.b && isParking && !stateB) {
                park.parkingStop();
                isParking = false;
            }

            stateB = gamepad1.b;
            stateA = gamepad1.a;
            telemetry.addData("ticks parking", park.upMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
