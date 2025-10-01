package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;



//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.DriveUtils;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;
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
    }
    public void start()
    {
        _limelight.start();
    }
    public void loop(){
        YawPitchRollAngles orientation = _IMU.getRobotYawPitchRollAngles();
        _limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = _limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            _telemetry.addData("Tx", llResult.getTx());
            _telemetry.addData("Ty", llResult.getTy());
            _telemetry.addData("Ta", llResult.getTa());

        }
    }
    //endregion

    //--- Arcade Drive with Speed Control

    //--- Directional Driving with D-Pad


    //endregion
}