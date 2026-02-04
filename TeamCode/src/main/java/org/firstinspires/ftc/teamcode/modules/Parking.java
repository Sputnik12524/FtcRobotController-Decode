package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Parking {
    public final DcMotor upMotor;

    public static double POWER = 1;

    public Parking(LinearOpMode linearOpMode) {
        this.upMotor = linearOpMode.hardwareMap.get(DcMotor.class, "park");
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //---------------------------------------------- MOVING BY POWER
    public void setParkingPowerUp() { upMotor.setPower(POWER); }
    public void setParkingPowerDown() { upMotor.setPower(-POWER); }
    public  void parkingStop(){
        upMotor.setPower(0);
    }

}
