package org.firstinspires.ftc.teamcode.opmodes.test;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Light", group = "test")
public class LightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower fl = Constants.createFollower(hardwareMap);
        Shooter sh = new Shooter(this, fl);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                sh.light1.setState(true);
                telemetry.addData("State of sensor", sh.light1.getState());
            } else if (gamepad1.b) {
                sh.light2.setState(true);
                telemetry.addData("Light 2 state", sh.light1.getState());
            } else {
                sh.light1.setState(false);
                sh.light2.setState(false);
            }
            telemetry.update();
        }
    }

}
