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

        left.setPosition(0.5);
        middle.setPosition(0.5);
        right.setPosition(0.5);
        double position = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            sleep(1000);
            if (gamepad1.a){
              position=position+0.1;
            }
            if (gamepad1.b){
                position=position-0.1;
            }
            if (gamepad1.x){
                position=position+0.01;
            }
            if (gamepad1.y){
                position=position-0.01;
            }
            left.setPosition(position);
            middle.setPosition(position);
            right.setPosition(position);

        }
        left.setPosition(0.5);
        middle.setPosition(0.5);
        right.setPosition(0.5);
        sleep(50);
    }
}