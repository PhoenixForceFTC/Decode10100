package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
@TeleOp(name="ServoTest", group="1")
@Config

public class ServoTest extends LinearOpMode {
    public Servo servo1;
    public Servo servo2;
    public static double position = 0.5;


    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "Pitch");
        servo2 = hardwareMap.get(Servo.class, "Yaw");

        telemetry.addData("Servo1 : ", servo1.getPosition());
        telemetry.addData("Servo2 : ", servo2.getPosition());

        waitForStart();

        telemetry.addLine();
        telemetry.update();

        while (!isStopRequested()) {

            while (opModeIsActive()) {
                servo1.setPosition(position);
                servo2.setPosition(position);

                telemetry.addData("Servo 1 Position >", servo1.getPosition());
                telemetry.addData("Servo 2 Position >", servo2.getPosition());
                telemetry.update();
                waitForStart();
            }
        }
    }
}