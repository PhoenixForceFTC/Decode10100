package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.acmerobotics.dashboard.config.Config;


import java.util.ArrayList;
import java.util.List;
@Config
public class LimelightHardware2Axis
{
    //region --- Constants ---
    //endregion

    //region --- Variables ---
    private double _cameraTiltAngle = 0.0; // Camera tilt angle in degrees (positive = tilted up)
    private double _yawPositionStart = 0.5;
    private double _pitchPositionStart = 0.6;
    private double _yawPositionStartAutoBack = 0.5;
    private double _pitchPositionStartAuto = 0.5;
    private double _yawPosition = _yawPositionStart;
    private double _pitchPosition = _pitchPositionStart;

    //endregion

    // Define an enum representing the days of the week
    public enum Motif {
        PPG,
        PGP,
        GPP;

    }



    //region --- Hardware ---
    private final Gamepad _gamepad;
    private final IMU _IMU;
    private final Limelight3A _limelight;
    private final Telemetry _telemetry;
    private final boolean _showInfo;

    private final Servo _yaw;
    private final Servo _pitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private int _robotVersion;
    //endregion

    //region --- Constructor ---
    public LimelightHardware2Axis(IMU imu, Limelight3A limelight,
                                  Gamepad gamepad, Telemetry telemetry,Servo yaw,Servo pitch, int robotVersion, boolean showInfo)
    {
        _IMU= imu;
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,135,0,0,0));
        _IMU.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        _limelight = limelight;
        _limelight.pipelineSwitch(0);
        _gamepad = gamepad;
        _telemetry = telemetry;
        _robotVersion = robotVersion;
        _showInfo = showInfo;
        _limelight.start();
        _yaw=yaw;
        _pitch=pitch;
    }

    /**
     * Set the camera tilt angle
     * @param angle Tilt angle in degrees (positive = tilted up, negative = tilted down)
     */
    public void setCameraTiltAngle(double angle) {
        _cameraTiltAngle = angle;
    }

    /**
     * Get the current camera tilt angle
     * @return Camera tilt angle in degrees
     */
    public double getCameraTiltAngle() {
        return _cameraTiltAngle;
    }

    /**
     * Calculate the horizontal floor distance to an AprilTag using camera tilt angle
     * Uses the tag's 3D position relative to robot and projects it onto the floor plane
     * @param llResult The Limelight result containing target data
     * @return Floor distance to the tag, or -1 if calculation fails
     */
    private double calculateFloorDistance(LLResult llResult) {
        if (llResult == null || !llResult.isValid()) {
            return -1;
        }

        // Get fiducial results (AprilTag detections)
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return -1;
        }

        // Get the first detected AprilTag
        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

        // Get the tag's position relative to the robot/camera
        Pose3D tagPose = fiducial.getTargetPoseCameraSpace();

        if (tagPose == null) {
            return -1;
        }

        // Get the 3D position components
        double x = tagPose.getPosition().x; // Lateral (left/right)
        double y = tagPose.getPosition().y; // Vertical (up/down)
        double z = tagPose.getPosition().z; // Forward (depth)

        // Calculate the straight-line distance from camera to tag
        double straightLineDistance = Math.sqrt(x * x + y * y + z * z);

        // Convert camera tilt angle to radians
        double tiltRad = Math.toRadians(_cameraTiltAngle);

        // Project the straight-line distance onto the floor plane
        // If camera is tilted up (positive angle), multiply by cos(tiltAngle)
        double floorDistance = straightLineDistance * Math.cos(tiltRad);

        return floorDistance;
    }

    /**
     * Get detailed distance information
     * @param llResult The Limelight result
     * @return Array [floorDistance, forwardDistance, lateralDistance, verticalOffset] or null if no target
     */
    private double[] getDistanceBreakdown(LLResult llResult) {
        if (llResult == null || !llResult.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);
        Pose3D tagPose = fiducial.getTargetPoseCameraSpace();

        if (tagPose == null) {
            return null;
        }

        double x = tagPose.getPosition().x; // Lateral (side to side)
        double y = tagPose.getPosition().y; // Vertical (up/down)
        double z = tagPose.getPosition().z; // Forward (depth)

        double straightLineDistance = Math.sqrt(x * x + y * y + z * z);
        double tiltRad = Math.toRadians(_cameraTiltAngle);
        double floorDistance = straightLineDistance * Math.cos(tiltRad);

        return new double[] {floorDistance, z, x, y};
    }

    public void start()
    {

    }
    public void servos(){
        _yaw.setPosition(_yawPosition);
        _pitch.setPosition(_pitchPosition);
        _cameraTiltAngle=(_pitchPosition-0.5)*300;
    }
    public void setServos(double yawPosition, double pitchPosition){
        _yawPosition=yawPosition;
        pitchPosition=pitchPosition;
    };


    public Motif getObliskTagId(){
        return Motif.GPP;
    }

    private void readObeliskAuto(){
        _yaw.setPosition(_yawPositionStartAutoBack);
        _pitch.setPosition(_pitchPositionStartAuto);

    }

    private LLResultTypes.FiducialResult getFiducial(String targetTagIDs)
    {
        LLResult llResult = _limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // Get the first fiducial
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int tagId = fiducial.getFiducialId();
                    if (targetTagIDs.contains(Integer.toString(tagId))) {
                        return fiducial;
                    }
                }
            }
        }

        return null;

    }
    public void loop(){
        YawPitchRollAngles orientation = _IMU.getRobotYawPitchRollAngles();
        _limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = _limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Get the first fiducial
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int tagId = fiducial.getFiducialId();

                // Calculate floor distance
                double floorDistance = calculateFloorDistance(llResult);

                // Get detailed breakdown
                double[] breakdown = getDistanceBreakdown(llResult);

                _telemetry.addLine("=== Tag ID: " + tagId + " ===");

                if (floorDistance > 0 && breakdown != null) {
                    _telemetry.addData("Floor Distance", String.format("%.2f m", floorDistance));
                    _telemetry.addData("Forward (Z)", String.format("%.2f m", breakdown[1]));
                    _telemetry.addData("Lateral (X)", String.format("%.2f m", breakdown[2]));
                    _telemetry.addData("Vertical (Y)", String.format("%.2f m", breakdown[3]));
                } else {
                    _telemetry.addData("Floor Distance", "Cannot calculate");
                }

                _telemetry.addData("Camera Tilt", String.format("%.2f°", _cameraTiltAngle)+"°");

                // Show raw angles too
                double tx = llResult.getTx();
                double ty = llResult.getTy();
                _telemetry.addData("Tx (Horizontal)", String.format("%.2f°", tx));
                _telemetry.addData("Ty (Vertical)", String.format("%.2f°", ty));
            } else {
                _telemetry.addLine("No AprilTags detected");
            }

            // Get robot pose if available
            Pose3D botPose = llResult.getBotpose();
            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double yaw = botPose.getOrientation().getYaw();

                _telemetry.addLine("--- Robot Pose ---");
                _telemetry.addData("Position", String.format("(%.2f, %.2f)", x, y));
                _telemetry.addData("Yaw", String.format("%.2f°", yaw));
            }
        } else {
            _telemetry.addLine("No valid Limelight result");
        }
    }
    public void setPipeline(int index){
        _limelight.pipelineSwitch(index);
    }
    public List<LLResultTypes.FiducialResult> getVisibleTags() {
        LLResult llResult = _limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return null;
        return llResult.getFiducialResults();
    }
    public boolean fiducialResultsContain(int id){
        List<Integer> ids = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : getVisibleTags()) {
            ids.add(f.getFiducialId());
        }
        return ids.contains(id);

    }
    //endregion
}