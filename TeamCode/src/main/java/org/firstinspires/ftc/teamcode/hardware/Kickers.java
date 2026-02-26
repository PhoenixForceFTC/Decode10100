package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.DataLog;
import org.firstinspires.ftc.teamcode.utils.DataLogger;
//endregion

@Config
public class Kickers
{

    FtcDashboard dashboard = FtcDashboard.getInstance();


    private final Gamepad _gamepad;
    private final Telemetry _telemetry;

    private final double kickedL = 0.3;
    private final double zeroL = 0.6;

    private final double kickedM = 0.3;
    private final double zeroM = 0.6;

    private final double kickedR = 0.7;
    private final double zeroR = 0.4;

    // delay before going to zero position
    private final double KICKER_ACTION_DELAY = 1.0;
    private final double GLOBAL_ACTION_DELAY = 0.1;

    private Servo _kickerLeft;
    private Servo _kickerMid;
    private Servo _kickerRight;

    private ElapsedTime timerL = new ElapsedTime(2);
    private ElapsedTime timerM = new ElapsedTime(2);
    private ElapsedTime timerR = new ElapsedTime(2);
    private ElapsedTime timerGlobal = new ElapsedTime(2);


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

    public boolean run(double targetSpeed, double speed, boolean run){
        boolean kicked =false;
        if(speed/targetSpeed>0.9 && run) {
            if (_gamepad.dpad_left) {
                fireKicker(0);
            }
            if (_gamepad.dpad_up) {
                fireKicker(1);
            }
            if (_gamepad.dpad_right) {
                fireKicker(2);
            }
            if (_gamepad.dpad_down){
                fireKicker(3);
            }

        }
        if(timerL.seconds() < KICKER_ACTION_DELAY){
            _kickerLeft.setPosition(kickedL);
            kicked = true;
        }else{
            _kickerLeft.setPosition(zeroL);
        }

        if(timerM.seconds() < KICKER_ACTION_DELAY){
            _kickerMid.setPosition(kickedM);
            kicked = true;
        }else{
            _kickerMid.setPosition(zeroM);
        }

        if(timerR.seconds() < KICKER_ACTION_DELAY){
            _kickerRight.setPosition(kickedR);
            kicked = true;
        }else{
            _kickerRight.setPosition(zeroR);
        }
        return kicked;
    }

    public boolean runFinal(double targetSpeed, double speed, boolean run,double targetSpeed3Ball){
        boolean kicked =false;
        if(speed/targetSpeed>0.9 && run) {
            if (_gamepad.dpad_left) {
                fireKicker(0);
            }
            if (_gamepad.dpad_up) {
                fireKicker(1);
            }
            if (_gamepad.dpad_right) {
                fireKicker(2);
            }

        }
        if(speed/targetSpeed3Ball>0.9 && run){
            if (_gamepad.dpad_down){
                fireKicker(3);
            }
        }
        if(timerL.seconds() < KICKER_ACTION_DELAY){
            _kickerLeft.setPosition(kickedL);
            kicked = true;
        }else{
            _kickerLeft.setPosition(zeroL);

        }

        if(timerM.seconds() < KICKER_ACTION_DELAY){
            _kickerMid.setPosition(kickedM);
            kicked = true;
        }else{
            _kickerMid.setPosition(zeroM);
        }

        if(timerR.seconds() < KICKER_ACTION_DELAY){
            _kickerRight.setPosition(kickedR);
            kicked = true;
        }else{
            _kickerRight.setPosition(zeroR);
        }
        return kicked;
    }
    public DataLog.Shooter run2(double targetSpeed, double speed, boolean run){
        DataLog.Shooter shooter = DataLog.Shooter.Unkown;
        if(speed/targetSpeed>0.9 && run) {
            if (_gamepad.dpadLeftWasPressed()) {
                fireKicker(0);
                shooter = DataLog.Shooter.Left;
            }
            if (_gamepad.dpadUpWasPressed()) {
                fireKicker(1);
                shooter = DataLog.Shooter.Middle;
            }
            if (_gamepad.dpadRightWasPressed()) {
                fireKicker(2);
                shooter = DataLog.Shooter.Right;
            }
            if (_gamepad.dpadDownWasPressed()){
                fireKicker(3);
                shooter = DataLog.Shooter.All;
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
        return shooter;
    }

    public int[] fireAutoKickerSeq(LimelightHardware2Axis.Motif targetMotif, LimelightHardware2Axis.Motif intakeMotif)
    {
        char[] targetMotifSeq = targetMotif.toString().toCharArray();
        char[] intakeMotifSeq = intakeMotif.toString().toCharArray();
        int[] seqArr = new int[3];
        int count = 0;
        for (char c : targetMotifSeq) {
            int pos = findChar(intakeMotifSeq, c);
            if (pos>-1 && pos<3) {
                seqArr[count++]=pos;
                intakeMotifSeq[pos] = 'x';
            }
        }

        return seqArr;
    }

    private static int findChar(char[] arr, char target) {
        if (arr == null) {
            return -1; // handle null array
        }

        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == target) {
                return i; // found, return index
            }
        }

        return -1; // not found
    }

    // 0 = leftmost kicker, 1 = middle kicker, 2 = rightmost kicker, 3 = all kickers
    public void fireKicker(int kickerPos)
    {

            if (kickerPos == 0 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY) {
                timerL.reset();
                timerGlobal.reset();
            }

            else if (kickerPos==1 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY) {
                timerM.reset();
                timerGlobal.reset();
            }

            else if (kickerPos==2 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY) {
                timerR.reset();
                timerGlobal.reset();
            }

            else if (kickerPos==3 && timerGlobal.seconds() > GLOBAL_ACTION_DELAY) {
                timerL.reset();
                timerM.reset();
                timerR.reset();
                timerGlobal.reset();
            }

    }

    public void fireKickerAuto(int kickerPos)
    {

        if (kickerPos == 0){
            _kickerLeft.setPosition(kickedL);
        }
        else if (kickerPos == 1){
            _kickerMid.setPosition(kickedM);
        }
        else if (kickerPos == 2){
            _kickerRight.setPosition(kickedR);
        }
        else if (kickerPos == 3) {
            _kickerLeft.setPosition(kickedL);
            _kickerMid.setPosition(kickedM);
            _kickerRight.setPosition(kickedR);
        }
    }

    public void retractKickerAuto(int kickerPos)
    {

        if (kickerPos == 0){
            _kickerLeft.setPosition(zeroL);
        }
        else if (kickerPos == 1){
            _kickerMid.setPosition(zeroM);
        }
        else if (kickerPos == 2){
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