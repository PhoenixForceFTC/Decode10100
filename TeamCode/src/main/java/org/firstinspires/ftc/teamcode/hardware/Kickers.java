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

    //region --- Hardware ---
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Gamepad _gamepad;
    private final Telemetry _telemetry;
    //endregion

    //region --- Variables
    private Servo _kickerLeft;
    private Servo _kickerMid;
    private Servo _kickerRight;

    //--- Cooldown timers for kickers
    private ElapsedTime timerL = new ElapsedTime(2);
    private ElapsedTime timerM = new ElapsedTime(2);
    private ElapsedTime timerR = new ElapsedTime(2);

    //--- Cooldown timer for input
    private ElapsedTime timerGlobal = new ElapsedTime(2);

    private final Boolean _showInfo;
    //endregion

    //region --- Constants ---
    private final double kickedL = 0.3;
    private final double zeroL = 0.5;

    private final double kickedM = 0.3;
    private final double zeroM = 0.5;

    private final double kickedR = 0.7;
    private final double zeroR = 0.5;

    //--- Delay before going to zero position
    private final double KICKER_ACTION_DELAY = 1.0;
    //--- Delay before we can kick again
    private final double GLOBAL_ACTION_DELAY = 0.5;
    //endregion




    //region --- Constructor ---
    public Kickers(Servo kickerL, Servo kickerM, Servo kickerR, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        _kickerLeft = kickerL;
        _kickerMid = kickerM;
        _kickerRight = kickerR;
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }
    //endregion

    //--- Allows for kicking
    //--- targetSpeed and speed refer to shooters
    public void run(double targetSpeed, double speed){

        //--- Allows for reset of timers when shooter reaches speed
        if(speed/targetSpeed>0.9 && speed/targetSpeed<1.01) {

            //--- Resets timer for left kicker and for action cooldown
            //--- When input is available and action cooldown over
            if (_gamepad.dpad_left && timerGlobal.seconds() < GLOBAL_ACTION_DELAY) {
                timerL.reset(); //--- timer for left servo
                timerGlobal.reset(); //--- So that we don't kick something else immediately after
            }

            else if (_gamepad.dpad_up && timerGlobal.seconds() < GLOBAL_ACTION_DELAY) {
                timerM.reset();
                timerGlobal.reset();
            }

            else if (_gamepad.dpad_right && timerGlobal.seconds() < GLOBAL_ACTION_DELAY) {
                timerR.reset();
                timerGlobal.reset();
            }
        }

        //--- If timer is reset, the servo is in kicked position
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

    //--- Get telemetry data
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