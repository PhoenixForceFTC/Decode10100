package org.firstinspires.ftc.teamcode.testOpModes;

//region --- Imports ---

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Kickstand;
//endregion

@TeleOp(name="Kicker Test", group="1")
public class Kicker_Test extends LinearOpMode
{
    public Servo left = null;
    public Servo middle = null;
    public Servo right = null;

    //public Kickstand kickstand;
    //private boolean kicked=false;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(Servo.class, "KL");
        middle = hardwareMap.get(Servo.class, "KM");
        right = hardwareMap.get(Servo.class, "KR");

        waitForStart();

        while (opModeIsActive()) {
            left.setPosition(0.5);
            middle.setPosition(0.5);
            right.setPosition(0.5);
        }
        sleep(50);
    }
}