package org.firstinspires.ftc.teamcode.testOpModes;

//region --- Imports ---

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Kickstand;


import org.firstinspires.ftc.robotcore.external.Telemetry;
//endregion

@Config
public class Kickstand_Test extends LinearOpMode
{
    private LinearOpMode myOpMode = null;
    public DcMotorEx motorKickstand = null;

    public Kickstand kickstand;

    @Override
    public void runOpMode() throws InterruptedException {

        motorKickstand = myOpMode.hardwareMap.get(DcMotorEx.class, "kickstand");
        kickstand = new Kickstand(motorKickstand,
                myOpMode.gamepad2,
                myOpMode.telemetry,
                false);

        while (opModeIsActive()) {
            kickstand.run();
        }

        sleep(50);
    }
}