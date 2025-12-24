package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.Timer;

@Config
@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "3")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Sorting st;
    Limelight ll;
    Logger logger;
    ElapsedTime timer;

    public static double RPS = 25; //Maximum = ~52 rps for old shooter

    boolean stateA1 = false;
    boolean stateB1 = false;
    boolean stateX1 = false;
    boolean stateY1 = false;
    boolean isRotateIn = false;
    boolean isRotateOut = false;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        logger = new Logger("ShVelocityTime");
        sh = new Shooter(this);
        ll = new Limelight(this);
        in = new Intake(this);
        st = new Sorting(this);


        st.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        MotorConfigurationType motorConfigurationType = sh.shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        sh.shooter.setMotorType(motorConfigurationType);

        timer.reset();
        logger.addHeader("Time,Velocity");

        waitForStart();
        while (opModeIsActive()) {

            /// Shooter
            if (gamepad1.y && !stateY1) {
                sh.shootByVelocity(RPS);
            } else if (gamepad1.x && !stateX1) {
                sh.shootStop();
            }
            stateY1 = gamepad1.y;
            stateX1 = gamepad1.x;

            /// Intake
            if (gamepad1.a && !isRotateIn && !stateA1) {
                in.rotateIn();
                isRotateIn = true;
                isRotateOut = false;
            } else if (gamepad1.a && isRotateIn && !stateA1) {
                in.rotateStop();
                isRotateIn = false;
            }
            if (gamepad1.b && !isRotateOut && !stateB1) {
                in.rotateOut();
                isRotateOut = true;
                isRotateIn = false;
            } else if (gamepad1.b && isRotateOut && !stateB1) {
                in.rotateStop();
                isRotateOut = false;
            }
            stateA1 = gamepad1.a;
            stateB1 = gamepad1.b;

            /// Sorting
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                st.drumTeleGo();
            } else {
                st.drumStop();
            }
            /// Walls
            if (gamepad2.dpad_down) {
                st.verticalWallOpen();
            } else if (gamepad2.dpad_up) {
                st.verticalWallClose();
            }
            if (gamepad2.dpad_left) {
                st.horizontalWallOpen();
            } else if (gamepad2.dpad_right) {
                st.horizontalWallClose();
            }

            logger.addLine(timer.milliseconds(), sh.getVelocityRPS());

            telemetry.addData("real PRS", sh.getVelocityRPS());
            telemetry.addData("real TPS", sh.getVelocityTPS());
            telemetry.update();

            dashTele.addLine("SHOOTER:");
            dashTele.addData("target of RPS:", RPS);
            dashTele.addData("real RPS:", sh.getVelocityRPS());
            dashTele.addData("real TPS:", sh.getVelocityTPS());
            dashTele.addData("Value of encoders:", sh.shooter.getCurrentPosition());
//            dashTele.addLine("INTAKE:");
//            dashTele.addData("real RPS:", in.getVelocityRPS());
//            dashTele.addData("real TPS:", in.getVelocityTPS());
//            dashTele.addData("Value of encoders:", in.catcher.getCurrentPosition());
            dashTele.update();
        }
        logger.fileClose();
    }
}


