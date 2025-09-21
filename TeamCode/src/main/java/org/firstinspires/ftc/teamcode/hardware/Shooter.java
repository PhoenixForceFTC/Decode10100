package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;
//endregion

public class Shooter
{




    //region --- Hardware ---
    private final DcMotorEx _motorShooterLeft;
    private final DcMotorEx _motorShooterRight;
    private final Gamepad _gamepad;
    private final Telemetry _telemetry;

    private final Boolean _showInfo;

    public static Integer ticksPerRotation = 28;
    //endregion

    //region --- Constructor
    public Shooter(DcMotorEx motorShooterLeft, DcMotorEx motorShooterRight, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        this._motorShooterLeft = motorShooterLeft;
        this._motorShooterRight = motorShooterRight;
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }


    public void initialize()
    {

    }
    public void shoot(){
        _motorShooterLeft.setVelocity(3000*ticksPerRotation);
        _motorShooterRight.setVelocity(3000*ticksPerRotation);
    }


    public void getTelemetry(){
        _telemetry.addData("Lift -> Target Position Left", _motorShooterLeft.getTargetPosition());
        _telemetry.addData("Lift -> Target Position Right", _motorShooterRight.getTargetPosition());
        _telemetry.addData("Lift -> Current Position Left", MotorUtils.getCurrentPosition(_motorShooterLeft));
        _telemetry.addData("Lift -> Current Position Right", MotorUtils.getCurrentPosition(_motorShooterRight));
    }


}