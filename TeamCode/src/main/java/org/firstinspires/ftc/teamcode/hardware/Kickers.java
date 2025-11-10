package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class Kickers
{

    FtcDashboard dashboard = FtcDashboard.getInstance();


    private final Gamepad _gamepad;
    private final Telemetry _telemetry;

    private final double kickedL = 0.3;
    private final double zeroL = 0.5;

    private final double kickedM = 0.3;
    private final double zeroM = 0.5;

    private final double kickedR = 0.7;
    private final double zeroR = 0.5;

    // delay before going to zero position
    private final double KICKER_ACTION_DELAY = 1.0;

    private Servo _kickerLeft;
    private Servo _kickerMid;
    private Servo _kickerRight;

    private ElapsedTime timerL = new ElapsedTime(2);
    private ElapsedTime timerM = new ElapsedTime(2);
    private ElapsedTime timerR = new ElapsedTime(2);


    //endregion
    private final Boolean _showInfo;


    //region --- Constructor
    public Kickers(Servo kickerL, Servo kickerM, Servo kickerR, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        _kickerLeft = kickerL;
        _kickerMid = kickerM;
        _kickerRight = kickerR;
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }

    public void run(double targetSpeed, double speed){
        /*if(_gamepad.dpad_left){
            _kickerLeft.setPosition(kickedL);
        }else{
            _kickerLeft.setPosition(zeroL);
        }

        if(_gamepad.dpad_up){
            _kickerMid.setPosition(kickedM);
        }else{
            _kickerMid.setPosition(zeroM);
        }

        if(_gamepad.dpad_right){
            _kickerRight.setPosition(kickedR);
        }else{
            _kickerRight.setPosition(zeroR);
        }*/
        if(speed/targetSpeed>0.9) {
            if (_gamepad.dpad_left) {
                timerL.reset();
            }

            if (_gamepad.dpad_up) {
                timerM.reset();
            }

            if (_gamepad.dpad_right) {
                timerR.reset();
            }
        }
        if(timerL.seconds() < KICKER_ACTION_DELAY){
            _kickerLeft.setPosition(kickedL);
        }else{
            _kickerLeft.setPosition(zeroL);
        }

        if(timerM.seconds() < KICKER_ACTION_DELAY){
            _kickerMid.setPosition(kickedM);
        }else{
            _kickerMid.setPosition(zeroM);
        }

        if(timerR.seconds() < KICKER_ACTION_DELAY){
            _kickerRight.setPosition(kickedR);
        }else{
            _kickerRight.setPosition(zeroR);
        }
    }

    public void initialize()
    {
        //_motorShooterLeft.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
        //_motorShooterRight.setVelocityPIDFCoefficients(kP,kI ,kD , kF);
    }


    public void getTelemetry(){
        /*TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("this is a string line");
        packet.put("motorSpeed", _motorShooterLeft.getVelocity());
        packet.put("speed", speed);
        dashboard.sendTelemetryPacket(packet);*/

        if(_showInfo) {
            _telemetry.addData("Kicker 1 Position: ", _kickerLeft.getPosition());
            _telemetry.addData("Kicker 2 Position: ", _kickerMid.getPosition());
            _telemetry.addData("Kicker 3 Position: ", _kickerRight.getPosition());
        }
    }
}