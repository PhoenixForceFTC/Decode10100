package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.autos.AutoActions;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Back Auton")
public class Auto_Back extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts

        waitForStart();

        Actions.runBlocking(
                _robot.driveRR.mecanumDrive.actionBuilder(new Pose2d(-72, -24,0))
                        .splineToConstantHeading(new Vector2d(12, -12), )
                        .build());
        /*
        * Procedure (not pseudocode)
        * -Robot starts at back
        * -Scans obelisk for reading the motif
        * -moves to shooting area, faces goal
        * -Shoots
        * -Moves through back spike mark to intake artifacts
        * -moves back to shooting area
        * -shoots
        * -moves to loading zone, intakes artifacts
        * -moves back to shooting area
        * -shoots
        * */

    }
}
