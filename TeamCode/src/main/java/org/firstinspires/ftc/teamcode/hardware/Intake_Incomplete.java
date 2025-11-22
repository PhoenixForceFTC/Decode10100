package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class Intake_Incomplete
{




    //region --- Hardware ---
    private final DcMotorEx _intake;
    FtcDashboard _dashboard = FtcDashboard.getInstance();


    private final Gamepad _gamepad;
    private final Gamepad _gamepad2;
    public final Telemetry _telemetry;



    //endregion
    private final Boolean _showInfo;

    //public static Integer ticksPerRotation = 28;




    //region --- Constructor
    public Intake_Incomplete( DcMotorEx intake, Gamepad gamepad,Gamepad gamepad2, Telemetry telemetry, boolean showInfo)
    {
        this._intake = intake;
        this._intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this._intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this._gamepad = gamepad;
        this._gamepad2 = gamepad2;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }
    //endregion

    //region --- Movement ---
    public void forward(){
        _intake.setPower(0.8);
    } //--- Inward movement of artifacts

    public void backward(){
        _intake.setPower(-0.8);
    } //--- Outward movement

    public void stop(){
        _intake.setPower(0);
    }
    //endregion

    //--- Uses controls to control intake, outtake, and stop
    public void run(){
        // change controls later
        if (_gamepad2.left_trigger>0.2){
            forward();
        }
        if (_gamepad2.right_trigger>0.2){
            backward();
        }
        if (_gamepad.y){
            stop();
        }
    }

    //--- Displays telemetry data
    public void getTelemetry(){
        _telemetry.addData("motor power: ", _intake.getPower());
    }

    //--- Gets speed of intake
    public double getSpeed(){
        return(_intake.getVelocity());
    }



}