package org.firstinspires.ftc.teamcode.opmodes.test.pidtuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;

@Config
@TeleOp
public class DetectorTest extends LinearOpMode{
    boolean isDetected = false;
    Shooter sh;
    Intake in;
    public static double VEL = 30;
    public void runOpMode(){
        sh = new Shooter(this);
        in = new Intake(this);
        boolean stateB = false;
        boolean stateX = false;
        boolean stateA = false;

        /// Intake
        boolean isRotateIn = false;
        boolean isShootingShort = false;
        boolean isShootingLong = false;
        boolean isRotateOut = false;
        boolean stateA1 = false;
        boolean stateB1 = false;

        /// Shooter
        boolean stateY1 = false;
        boolean stateX1 = false;
        boolean stateRB1 = false;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtele = dashboard.getTelemetry();
        Telemetry t = new MultipleTelemetry(telemetry, dashtele);

        waitForStart();

        while(opModeIsActive()){
            if(sh.isDetected() && !isDetected){
                isDetected = true;
            }

            if(gamepad1.b && !stateB){
                sh.setVelocityTarget(VEL);
                sh.setShortThrowMode();
                sh.shootByVelocity();
            }
            if(gamepad1.x && !stateX) sh.shootStop();

            // INTAKE
            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                in.transferSetPower(Intake.TRANSFER_POWER);
                isRotateIn = true;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                in.transferSetPower(0);
                isRotateIn = false;
            }

            if(gamepad1.dpad_down) isDetected = false;

            stateB = gamepad1.b;
            stateA = gamepad1.a;
            stateX = gamepad1.x;
            t.update();
            t.addData("RPS: ", sh.getVelocityRPS());
            t.addData("SpinUp: ", sh.isSpinUp());
            t.addData("isDetect: ", isDetected);
        }
    }

}
