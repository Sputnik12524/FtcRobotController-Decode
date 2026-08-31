package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
public class Intake {

    private final DcMotorEx catcher;

    public static double POWER_CATCHER = 1;
    public static double POWER_SERVO = 1;
    public double voltage;

    public Intake(LinearOpMode opMode) {
        this.catcher = opMode.hardwareMap.get(DcMotorEx.class, "catcher");
    }

    //---------------------------------------------- ROTATE

    public void rotateIn() {
        catcher.setPower(POWER_CATCHER);
    }
    public void rotateOut() {
        catcher.setPower(-POWER_CATCHER);
    }
    public void rotateStop() {
        catcher.setPower(0);
    }

    public double getAmps(){
        return catcher.getCurrent(CurrentUnit.AMPS);
    }

}
