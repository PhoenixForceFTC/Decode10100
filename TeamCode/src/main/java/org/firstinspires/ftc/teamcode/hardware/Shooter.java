package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorUtils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import kotlin.reflect.KDeclarationContainer;
//endregion

@Config
public class Shooter
{




    //region --- Hardware ---
    private final DcMotorEx _motorShooterLeft;
    FtcDashboard dash = FtcDashboard.getInstance();
    private final DcMotorEx _motorShooterRight;
    private final Gamepad _gamepad;
    private final Telemetry _telemetry;
    //endregion
    private final Boolean _showInfo;

    public static Integer ticksPerRotation = 28;

    public final double speed;


    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    //defualt pidf is p:10 i:3 d:0 f:0


    //region --- Constructor
    public Shooter(DcMotorEx motorShooterLeft, DcMotorEx motorShooterRight, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        this._motorShooterLeft = motorShooterLeft;
        this._motorShooterRight = motorShooterRight;
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
        speed = 0;
    }


    public void initialize()
    {
        //_motorShooterLeft.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
    }
    public void shoot(int speed){

        _motorShooterLeft.setVelocity(speed*ticksPerRotation/60);
        _motorShooterRight.setVelocity(speed*ticksPerRotation/60);
        speed = speed*ticksPerRotation/60;
    }


    public void getTelemetry(){
        _telemetry.addData("pidf coeficients for get velocoity", _motorShooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("this is a string line");
        packet.put("motorSpeed", _motorShooterLeft.getVelocity());
        packet.put("speed", speed);
        dash.sendTelemetryPacket(packet);
    }
    public double getSpeed(){
        return(_motorShooterLeft.getVelocity());
    }



}