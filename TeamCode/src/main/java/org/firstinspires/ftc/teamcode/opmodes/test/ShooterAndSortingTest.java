package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.opmodes.test.pidtuners.VeloPIDTuner.MOTOR_VELO_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@Config
@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "3")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Sorting st;
    Limelight ll;
    private VoltageSensor batteryVoltageSensor;

    public static double RPS = 10; //Maximum = ~52 rps

    boolean stateA1 = false;
    boolean stateB1 = false;
    boolean stateX1 = false;
    boolean stateY1 = false;
    boolean isRotateIn = false;
    boolean isRotateOut = false;

    @Override
    public void runOpMode() {
        sh = new Shooter(this);
        ll = new Limelight(this);
        in = new Intake(this);
        st = new Sorting(this);

        st.drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sh.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sh.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        setPIDFCoefficients(sh.shooter, MOTOR_VELO_PID);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        MotorConfigurationType motorConfigurationType = sh.shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        sh.shooter.setMotorType(motorConfigurationType);

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

            dashTele.addLine("SHOOTER:");
            dashTele.addData("target of RPS:", RPS);
            dashTele.addData("real RPS:", sh.getVelocityRPS());
            dashTele.addData("real TPS:", sh.getVelocityTPS());
            dashTele.addData("Value of encoders:", sh.shooterTest.getCurrentPosition());
            dashTele.addLine("INTAKE:");
            dashTele.addData("real RPS:", in.getVelocityRPS());
            dashTele.addData("real TPS:", in.getVelocityTPS());
            dashTele.addData("Value of encoders:", in.catcher.getCurrentPosition());
            dashTele.update();
        }
    }
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}


