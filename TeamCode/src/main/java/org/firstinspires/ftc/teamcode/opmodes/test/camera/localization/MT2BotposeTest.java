package org.firstinspires.ftc.teamcode.opmodes.test.camera.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "TEST MT2")
@Config

public class MT2BotposeTest extends LinearOpMode {
    Limelight3A limelight3A;
    GoBildaPinpointDriver pinpoint;

    public static double x_Coordinate, y_Coordinate;

    @Override
    public void runOpMode() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        MultipleTelemetry t = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight3A.getStatus();

            LLResult result = limelight3A.getLatestResult();
            if (result.isValid()) {
                //Pose3D botposemt2 = result.getBotpose_MT2();

                Pose3D botpose = result.getBotpose();

                t.addData("botpose", botpose.toString()); //МЕТРЫ

                x_Coordinate = 144 - botpose.getPosition().x * 39.37 - 72; // неактуально САНТИМЕТРЫ //ЛОЖЬ
                y_Coordinate = 144 - -botpose.getPosition().y * 39.37 - 72;// неактуально ЛОЖЬ (см. вверх) //Теперь в дюймах дюймы сила

                t.addData("botpose y +  72: ", x_Coordinate);
                t.addData("botpose x + 72: ", y_Coordinate);
                Pose pp = new Pose(botpose.getPosition().y, botpose.getPosition().x, botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                        FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                t.addData("pose pp x", pp.getX()); //ДЮЙМЫ
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    t.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                limelight3A.updateRobotOrientation(pinpoint.getYawScalar());

            } else {
                t.addLine("No data available");
            }
            t.update();
        }
        limelight3A.stop();
    }

}
