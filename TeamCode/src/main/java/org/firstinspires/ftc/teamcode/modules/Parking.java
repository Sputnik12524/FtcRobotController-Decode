package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Parking {
    private DcMotor upMotor;

    public Parking (LinearOpMode linearOpMode){
        this.upMotor = linearOpMode.hardwareMap.get(DcMotor.class, "upMotor" );
    }
}
