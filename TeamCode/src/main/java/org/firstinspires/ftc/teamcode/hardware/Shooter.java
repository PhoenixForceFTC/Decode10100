package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import kotlin.reflect.KDeclarationContainer;
//endregion

@Config
public class Shooter
{




    //region --- Hardware ---
    private final DcMotorEx _motorShooterLeft;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    private final DcMotorEx _motorShooterRight;
    private final Gamepad _gamepad;
    public final Telemetry _telemetry;



    //endregion

    //region --- Variables ---
    private final Boolean _showInfo;

    public static Integer ticksPerRotation = 28;

    public double speed; // in ticks per second


    public static double kP = 700;
    public static double kI = 20;
    public static double kD = 60;
    public static double kF = 0;

    // Cached velocity — updated once per loop via cacheVelocity() to avoid redundant encoder reads.
    private double _cachedVelocity = 0;
    private final TelemetryPacket _telemetryPacket = new TelemetryPacket();
    //endregion

    //defualt pidf is p:10 i:3 d:0 f:0


    //region --- Constructor
    public Shooter(DcMotorEx motorShooterLeft, DcMotorEx motorShooterRight, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        this._motorShooterLeft = motorShooterLeft;
        this._motorShooterRight = motorShooterRight;
        this._motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this._motorShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this._motorShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this._motorShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this._motorShooterLeft.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        this._motorShooterRight.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
        speed = 0;
    }
    //endregion

    public void initialize()
    {
        //_motorShooterLeft.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
    }

    //--- get shooter into shooting abiltity
    public void shoot(int speed){

        //_motorShooterLeft.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        _motorShooterLeft.setVelocity(speed*ticksPerRotation/60);
        _motorShooterRight.setVelocity(speed*ticksPerRotation/60);

        this.speed = speed*ticksPerRotation/60;
    }

    /** Read and cache the motor velocity once per loop. Call at the top of every loop before getSpeed(). */
    public void cacheVelocity() {
        _cachedVelocity = _motorShooterLeft.getVelocity();
    }

    //--- Access telemetry data — reuses a pre-allocated packet to avoid GC pressure
    public void getTelemetry(){
        _telemetryPacket.put("pidf coeficients for get velocoity", _motorShooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        _telemetryPacket.addLine("this is a string line");
        _telemetryPacket.put("motorSpeed", _cachedVelocity);
        _telemetryPacket.put("speed", speed);
        dashboard.sendTelemetryPacket(_telemetryPacket);
    }

    //--- Get motor speed — returns cached value (call cacheVelocity() once per loop first)
    public double getSpeed(){
        return _cachedVelocity;
    }
    public double getSpeedRPM(){
        return _cachedVelocity * 60 / ticksPerRotation;
    }



}