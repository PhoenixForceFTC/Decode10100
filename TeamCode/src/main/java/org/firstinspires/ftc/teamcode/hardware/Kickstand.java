package org.firstinspires.ftc.teamcode.hardware;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class Kickstand
{
    FtcDashboard dashboard = FtcDashboard.getInstance();


    private final DcMotorEx _kickstand;
    private final Gamepad _gamepad;
    public final Telemetry _telemetry;



    //endregion

    //region --- Variables ---
    private final Boolean _showInfo;
    private final double COUNTS_PER_MOTOR_REV = 3895.9;

    private final int UNKICKED = 0;
    private final int KICKED = (int) (0.4*COUNTS_PER_MOTOR_REV);

    private Boolean button_was_pressed = false; //was button pressed?
    private Boolean to_kick = false;

    //endregion

    //region --- Constructor
    public Kickstand(DcMotorEx kickstand, Gamepad gamepad, Telemetry telemetry, boolean showInfo)
    {
        this._kickstand = kickstand;
        this._kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this._kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this._gamepad = gamepad;
        this._telemetry = telemetry;
        this._showInfo = showInfo;
    }
    //endregion

    public void run()
    {
        // when button pressed
        if(_gamepad.left_bumper && !button_was_pressed){
            to_kick = !to_kick; // if kicking
            button_was_pressed = true;
            _kickstand.setTargetPosition(to_kick ? KICKED : UNKICKED);
        } else if (!_gamepad.left_stick_button && button_was_pressed) {
            button_was_pressed = false;
        }

        // Check if the motor is not moving or the target position has changed
        if (!_kickstand.isBusy()) {
            _kickstand.setPower(1); // Move to the target position
        }

        // Stop the motor once it has reached the target position
        if (!_kickstand.isBusy()) {
            _kickstand.setPower(0.2); // Stop the motor
        }
    }


    //--- Access telemetry data
    public void getTelemetry(){
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
    }
}