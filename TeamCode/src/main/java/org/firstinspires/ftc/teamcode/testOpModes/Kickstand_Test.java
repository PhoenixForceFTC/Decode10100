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

@TeleOp(name="Kickstand Test", group="1")
public class Kickstand_Test extends LinearOpMode
{
    public DcMotor motorKickstand = null;

    public Kickstand kickstand;
    private boolean kicked=false;

    @Override
    public void runOpMode() throws InterruptedException {

        motorKickstand = hardwareMap.get(DcMotor.class, "kickstand");
        kickstand = new Kickstand(motorKickstand,
                gamepad2,
                telemetry,
                false);

        waitForStart();

        while (opModeIsActive()) {
            kicked = kickstand.run();
        }
        telemetry.addData("Kicked: ", kicked);
        sleep(50);
    }
}