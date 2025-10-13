package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public static double FORWARD;
    public static double POWER;
    public static double direction;


    public DriveTrain(LinearOpMode linearOpMode) {
        this.leftFront = linearOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        this.leftBack = linearOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        this.rightFront = linearOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        this.rightBack = linearOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
