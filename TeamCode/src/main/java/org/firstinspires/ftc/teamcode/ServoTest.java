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
    public static double position = 0.5;


    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "st");

        telemetry.addData("Servo1 : ", servo1.getPosition());

        waitForStart();

        telemetry.addLine();
        telemetry.update();

        while (!isStopRequested()) {

            while (opModeIsActive()) {
                servo1.setPosition(position);

                telemetry.addData("Position >", servo1.getPosition());
                telemetry.update();
                waitForStart();
            }
        }
    }
}