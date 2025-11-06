package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
//endregion

public class LimelightHardware2Axis
{
    //region --- Constants ---
    //--- Constants for Speed Multipliers
    //endregion

    //region --- Variables ---
    //endregion

    //region --- Hardware ---
    private final Gamepad _gamepad;
    private final IMU _IMU;
    private final Limelight3A _limelight;
    private final Telemetry _telemetry;
    private final boolean _showInfo;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    private int _robotVersion;
    //endregion

    //region --- Constructor ---
    public LimelightHardware2Axis(IMU imu, Limelight3A limelight,
                                  Gamepad gamepad, Telemetry telemetry, int robotVersion, boolean showInfo)
    {
        _IMU= imu;
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        _IMU.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        _limelight = limelight;
        _limelight.pipelineSwitch(0);
        _gamepad = gamepad;
        _telemetry = telemetry;
        _robotVersion = robotVersion;
        _showInfo = showInfo;
        _limelight.start();
    }
    public void start()
    {

    }
    public void loop(){
        YawPitchRollAngles orientation = _IMU.getRobotYawPitchRollAngles();
        _limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = _limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // you can get basic target info:
            double tx = llResult.getTx();
            double ty = llResult.getTy();
            // more importantly for pose:
            Pose3D botPose = llResult.getBotpose();
            if (botPose != null) {
                double x = botPose.getPosition().x;  // meters (or whatever units your field map uses)
                double y = botPose.getPosition().y;
                double z = botPose.getPosition().z;
                double roll  = botPose.getOrientation().getRoll();
                double pitch = botPose.getOrientation().getPitch();
                double yaw   = botPose.getOrientation().getYaw();
                _telemetry.addData("Robot X, Y", "(" + x + ", " + y + ")");
                _telemetry.addData("Robot Yaw", yaw);
            }
            // use botPose2 similarly if not null
        } else {
                _telemetry.addLine("No fiducials in fiducialResults()");
            }
//            _telemetry.addData("Tx", llResult.getTx());
//            _telemetry.addData("Ty", llResult.getTy());
//            _telemetry.addData("Ta", llResult.getTa());
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("Tx", llResult.getTx());
//            packet.put("Ty", llResult.getTy());
//            packet.put("Ta", llResult.getTa());
//            dashboard.sendTelemetryPacket(packet);


    }
    //endregion

    //--- Arcade Drive with Speed Control

    //--- Directional Driving with D-Pad


    //endregion
}