package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DriveUtilsAdvanced {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private double x = 0;
    private double dxdt = 0;
    private double y = 0;
    private double dydt = 0;
    private double heading = 0;
    private double yaw = 0;

    private Telemetry telemetry;

    public Drive drive;

    private MecanumDrive driveClass;
    public DriveUtilsAdvanced(HardwareMap hardwareMap, Pose2d pose, Drive drive, Telemetry telemetry){
        driveClass = new MecanumDrive(hardwareMap, pose);
        this.drive = drive;
        this.telemetry = telemetry;
    }


    public void setVars(double x, double dxdt, double y, double dydt, double heading, double yaw){
        this.x = x;
        this.dxdt = dxdt;
        this.y = y;
        this.dydt = dydt;
        this.heading = heading;
        this.yaw = yaw;
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("x speed", dxdt);
        telemetry.addData("y speed", dydt);
        telemetry.addData("heading", heading);
        telemetry.addData("heading speed", yaw);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("x speed", dxdt);
        packet.put("y speed", dydt);
        packet.put("heading", heading);
        packet.put("heading speed", yaw);
        dashboard.sendTelemetryPacket(packet);


    }
    public void setVarsAdvanced(double x, double y, double heading,double axial, double lateral, double yaw){
        setVars(x,lateral*sin(heading)+axial*cos(heading),y,axial*sin(heading)-lateral*cos(heading),heading,yaw);

    };
    public void driveMecanum(Gamepad gamepad){

        driveClass.localizer.update();
        setVarsAdvanced(
                driveClass.localizer.getPose().position.x,
                driveClass.localizer.getPose().position.y,
                driveClass.localizer.getPose().heading.toDouble(),
                -gamepad.left_stick_y * drive.getSpeedMultiplier(),
                gamepad.left_stick_x * drive.getSpeedMultiplier(),
                gamepad.right_stick_x * drive.getSpeedMultiplierRotate());
                //driveClass.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad.left_stick_x, gamepad.left_stick_y), gamepad.right_stick_x));
                //drive.driveControl(1);
                telemetry.addData("change in heading", thetadt());


                TelemetryPacket packet = new TelemetryPacket();
                Canvas c = packet.fieldOverlay();
                Drawing.drawRobot(c, driveClass.localizer.getPose());
                packet.put("change in heading", thetadt());

                dashboard.sendTelemetryPacket(packet);
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,thetadt());

                //if(thetadt() > 1){
                //    drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, 1);
                //} else if (thetadt()<1) {
                //    drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, -1);
                //} else{

                //}

    }

    public double thetadt (){ // radians per second hopefully

        return ((60+y)*dxdt-(60+x)*dydt)/((60+x)*(60+x)+(60+y)*(60+y));
    }



}
