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
    private final Telemetry _telemetry;



    //endregion
    private final Boolean _showInfo;

    public static Integer ticksPerRotation = 28;

    public double speed;


    public static double kP = 200;
    public static double kI = 10;
    public static double kD = 60;
    public static double kF = 0;

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


    public void initialize()
    {
        //_motorShooterLeft.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
    }
    public void shoot(int speed){

        _motorShooterLeft.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        _motorShooterRight.setVelocityPIDFCoefficients(kP, kI,kD , kF);
        _motorShooterLeft.setVelocity(speed*ticksPerRotation/60);
        _motorShooterRight.setVelocity(speed*ticksPerRotation/60);

        this.speed = speed*ticksPerRotation/60;
    }

    public void getTelemetry(){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("pidf coeficients for get velocoity", _motorShooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        packet.addLine("this is a string line");
        packet.put("motorSpeed", _motorShooterLeft.getVelocity());
        packet.put("speed", speed);
        dashboard.sendTelemetryPacket(packet);
    }
    public double getSpeed(){
        return(_motorShooterLeft.getVelocity());
    }



}