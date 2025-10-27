package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;



//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.DriveUtils;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;

import java.util.List;
//endregion

public class LimelightHardware
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
    public LimelightHardware(IMU imu, Limelight3A limelight,
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
        if(llResult != null && llResult.isValid()){
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    int id = f.getFiducialId();

                    // Choose the pose that suits you:
                    // - getRobotPoseTargetSpace() returns the robot pose relative to the tag (useful)
                    // - getTargetPoseRobotSpace() returns the tag pose in robot coords
                    Pose3D robotPose_relTag = f.getRobotPoseTargetSpace(); // returns Pose3D
                    Pose3D tagPose_relRobot = f.getTargetPoseRobotSpace(); // alternative

                    _telemetry.addLine("=== Tag ID: " + id + " ===");

                    if (robotPose_relTag != null) {
                        // Pose3D exposes a position object in the examples; print X/Y/Z
                        _telemetry.addData("RobotPose_relTag X (m)", "%.2f", robotPose_relTag.getPosition().x);
                        _telemetry.addData("RobotPose_relTag Y (m)", "%.2f", robotPose_relTag.getPosition().y);
                        _telemetry.addData("RobotPose_relTag Z (m)", "%.2f", robotPose_relTag.getPosition().z);

                        // Not all Pose3D wrappers expose roll/pitch/yaw the same way in examples.
                        // If your Pose3D has e.g. getRotation() or getYaw(), print them; otherwise print the whole pose.
                        try {
                            // try a common rotation accessor (may exist in your SDK version)
                            //_telemetry.addData("RobotPose_relTag rot", robotPose_relTag.getRotation().toString());
                        } catch (Throwable t) {
                            _telemetry.addData("RobotPose_relTag (raw)", robotPose_relTag.toString());
                        }
                    } else if (tagPose_relRobot != null) {
                        _telemetry.addData("TagPose_relRobot (raw)", tagPose_relRobot.toString());
                    } else {
                        _telemetry.addData("Pose", "null");
                    }
                }
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
    }
    //endregion

    //--- Arcade Drive with Speed Control

    //--- Directional Driving with D-Pad


    //endregion
}