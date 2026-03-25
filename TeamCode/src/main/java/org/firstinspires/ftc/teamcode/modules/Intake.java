package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
public class Intake {

    private final DcMotorEx catcher;
    private final CRServo transferServo;

    public static double POWER_CATCHER = 1;
    public static double POWER_SERVO = 1;
    public double voltage;

    public Intake(LinearOpMode opMode) {
        this.catcher = opMode.hardwareMap.get(DcMotorEx.class, "catcher");
        transferServo = opMode.hardwareMap.get(CRServo.class,"transferServo");
    }

    //---------------------------------------------- ROTATE

    public void rotateIn() {
        catcher.setPower(POWER_CATCHER);
        transferServo.setPower(POWER_SERVO);
    }
    public void rotateOut() {
        catcher.setPower(-POWER_CATCHER);
        transferServo.setPower(-POWER_SERVO);
    }
    public void rotateStop() {
        catcher.setPower(0);
        transferServo.setPower(0);
    }

    public double getVoltage(){
        return catcher.getCurrent(CurrentUnit.AMPS);
    }

}
