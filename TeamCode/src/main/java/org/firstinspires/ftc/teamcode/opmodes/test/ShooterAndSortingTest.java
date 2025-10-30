package org.firstinspires.ftc.teamcode.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;

@TeleOp(name = "TEST Shooter/Sorting", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {

    Shooter st;

    public void runOpMode() {
        st = new Shooter(this);

        st.setShortThrow();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                st.shoot();
            } else {
                st.shootStop();
            }

            if (gamepad1.right_bumper) {
                st.switchThrowMode();
            }

        }

    }
}

