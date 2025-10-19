package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class Kickers
{

    FtcDashboard dashboard = FtcDashboard.getInstance();


    private final Gamepad _gamepad;
    private final Telemetry _telemetry;

    private final double kicked = 0.1;
    private final double zero = 0;

    private Servo _kicker1;
    private Servo _kicker2;
    private Servo _kicker3;


    //endregion
    private final Boolean _showInfo;


    //region --- Constructor
    public Kickers(Servo kicker1, Servo kicker2, Servo kicker3, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        _kicker1 = kicker1;
        _kicker2 = kicker2;
        _kicker3 = kicker3;
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }

    public void run(){
        if(_gamepad.dpad_left){
            _kicker1.setPosition(kicked);
        }else{
            _kicker1.setPosition(zero);
        }

        if(_gamepad.dpad_up){
            _kicker2.setPosition(kicked);
        }else{
            _kicker2.setPosition(zero);
        }

        if(_gamepad.dpad_right){
            _kicker3.setPosition(kicked);
        }else{
            _kicker3.setPosition(zero);
        }
    }

    public void initialize()
    {
        //_motorShooterLeft.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
    }


    public void getTelemetry(){
        /*TelemetryPacket packet = new TelemetryPacket();
        packet.put("pidf coeficients for get velocoity", _motorShooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        packet.addLine("this is a string line");
        packet.put("motorSpeed", _motorShooterLeft.getVelocity());
        packet.put("speed", speed);
        dashboard.sendTelemetryPacket(packet);*/

        if(_showInfo) {
            _telemetry.addData("Kicker 1 Position: ", _kicker1.getPosition());
            _telemetry.addData("Kicker 2 Position: ", _kicker2.getPosition());
            _telemetry.addData("Kicker 3 Position: ", _kicker3.getPosition());
        }
    }
}