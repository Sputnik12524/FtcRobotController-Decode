package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Sorting;

@Config
@TeleOp(name = "TEST Shooter/Sorting/Intake", group = "Test")
public class ShooterAndSortingTest extends LinearOpMode {
    Shooter sh;
    Intake in;
    Sorting st;
    Limelight ll;

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

        sh.shooterTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sh.shooterTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

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


            dashTele.addLine("VELOCITY:");
            dashTele.addData("TPS:", sh.getVelocityTPS());
            dashTele.addData("RPS:", sh.getVelocityRPS());
            dashTele.addData("VALUE OF ENCODERS:", sh.shooterTest.getCurrentPosition());
            dashTele.update();
        }
    }
}


