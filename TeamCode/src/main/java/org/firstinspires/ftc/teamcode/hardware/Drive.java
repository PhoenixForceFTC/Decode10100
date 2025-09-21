package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.DriveUtils;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;
//endregion

public class Drive
{
    //region --- Constants ---
    //--- Constants for Speed Multipliers
    private static final double SPEED_FAST = 1.0;
    private static final double SPEED_SLOW = 0.5;
    private static final double SPEED_ROTATE_FAST = 0.8;
    private static final double SPEED_ROTATE_SLOW = 0.4;
    //endregion

    //region --- Variables ---
    private boolean _isSpeedFast = true; //--- Default movement speed mode
    private boolean _isRotateFast = false; //--- Default rotation speed mode
    private boolean _wasLeftStickButtonPressed = false;
    private boolean _wasRightStickButtonPressed = false;
    //endregion

    //region --- Hardware ---
    private final DcMotor _frontLeft;
    private final DcMotor _frontRight;
    private final DcMotor _rearLeft;
    private final DcMotor _rearRight;
    private final Gamepad _gamepad;
    private final Telemetry _telemetry;
    private final boolean _showInfo;

    private int _robotVersion;
    //endregion

    //region --- Constructor ---
    public Drive(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight,
                 Gamepad gamepad, Telemetry telemetry, int robotVersion, boolean showInfo)
    {
        _frontLeft = frontLeft;
        _frontRight = frontRight;
        _rearLeft = rearLeft;
        _rearRight = rearRight;
        _gamepad = gamepad;
        _telemetry = telemetry;
        _robotVersion = robotVersion;
        _showInfo = showInfo;
    }
    //endregion

    //--- Arcade Drive with Speed Control
    public void arcadeDriveSpeedControl()
    {
        if (_gamepad.left_stick_button && !_wasLeftStickButtonPressed)
        {
            _isSpeedFast = !_isSpeedFast; //--- Toggle movement speed
        }
        _wasLeftStickButtonPressed = _gamepad.left_stick_button;

        if (_gamepad.right_stick_button && !_wasRightStickButtonPressed)
        {
            _isRotateFast = !_isRotateFast; //--- Toggle rotation speed
        }
        _wasRightStickButtonPressed = _gamepad.right_stick_button;

        double speedMultiplier = _isSpeedFast ? SPEED_FAST : SPEED_SLOW;
        double speedMultiplierRotate = _isRotateFast ? SPEED_ROTATE_FAST : SPEED_ROTATE_SLOW;

        DriveUtils.arcadeDrive(_frontLeft, _frontRight, _rearLeft, _rearRight, _gamepad, _telemetry, _showInfo, speedMultiplier, speedMultiplierRotate);

        if (_showInfo)
        {
            _telemetry.addData("Drive -> Speed Mode", _isSpeedFast ? "FAST" : "SLOW");
            _telemetry.addData("Drive -> Rotate Mode", _isRotateFast ? "FAST" : "SLOW");
        }
    }

    //--- Directional Driving with D-Pad
    public void directionDrive(double speed)
    {
        if (_gamepad.dpad_up)
        {
            moveForward(speed);
            if (_showInfo) _telemetry.addData("Drive -> Direction", "FORWARD (%4.2f)", speed);
        }
        else if (_gamepad.dpad_down)
        {
            moveBackward(speed);
            if (_showInfo) _telemetry.addData("Drive -> Direction", "BACKWARD (%4.2f)", speed);
        }
        else if (_gamepad.dpad_left)
        {
            moveLeft(speed);
            if (_showInfo) _telemetry.addData("Drive -> Direction", "LEFT (%4.2f)", speed);
        }
        else if (_gamepad.dpad_right)
        {
            moveRight(speed);
            if (_showInfo) _telemetry.addData("Drive -> Direction", "RIGHT (%4.2f)", speed);
        }
        else
        {
            //--- Stop motors if no D-Pad button is pressed
            stopMotors();
            if (_showInfo) _telemetry.addData("Drive -> Direction", "STOP");
        }
    }

    //--- Master drive control method to choose the active driving mode
    public void driveControl(double speed)
    {
        //--- If any D-Pad button is pressed, use directional drive; otherwise, use arcade drive
        if (_gamepad.dpad_up || _gamepad.dpad_down || _gamepad.dpad_left || _gamepad.dpad_right)
        {
            directionDrive(speed);
        }
        else
        {
            arcadeDriveSpeedControl();
        }
    }

    //region --- Move Directions ---

    //--- Moves the robot forward
    private void moveForward(double speed)
    {
        MotorUtils.setPower(_frontLeft, speed);
        MotorUtils.setPower(_frontRight, speed);
        MotorUtils.setPower(_rearLeft, speed);
        MotorUtils.setPower(_rearRight, speed);
    }

    //--- Moves the robot backward
    private void moveBackward(double speed)
    {
        MotorUtils.setPower(_frontLeft, -speed);
        MotorUtils.setPower(_frontRight, -speed);
        MotorUtils.setPower(_rearLeft, -speed);
        MotorUtils.setPower(_rearRight, -speed);
    }

    //--- Moves the robot to the left
    private void moveLeft(double speed)
    {
        MotorUtils.setPower(_frontLeft, -speed);
        MotorUtils.setPower(_frontRight, speed);
        MotorUtils.setPower(_rearLeft, speed);
        MotorUtils.setPower(_rearRight, -speed);
    }

    //--- Moves the robot to the right
    private void moveRight(double speed)
    {
        MotorUtils.setPower(_frontLeft, speed);
        MotorUtils.setPower(_frontRight, -speed);
        MotorUtils.setPower(_rearLeft, -speed);
        MotorUtils.setPower(_rearRight, speed);
    }

    //--- Stops all drive motors
    private void stopMotors()
    {
        MotorUtils.setPower(_frontLeft, 0);
        MotorUtils.setPower(_frontRight, 0);
        MotorUtils.setPower(_rearLeft, 0);
        MotorUtils.setPower(_rearRight, 0);
    }

    //endregion
}