package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;


public class DriveUtilsAdvanced {
    private List<Action> runningActions = new ArrayList<>();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean isBlue = false;
    private boolean isAligning = false;
    private double x = 0;

    private double x2 = 60;
    private double x3 = 66;
    private double dxdt = 0;
    private double y = 0;
    private double y2 = 60;
    private double y3 = 60;
    private double dydt = 0;
    private double heading = 0;
    private double yaw = 0;

    private Telemetry telemetry;

    public Drive drive;

    private MecanumDrive driveClass;

    private LimelightHardware2Axis limelightHardware2Axis;
    public DriveUtilsAdvanced(HardwareMap hardwareMap, Pose2d pose, Drive drive,LimelightHardware2Axis limelightHardware2Axis, Telemetry telemetry){
        driveClass = new MecanumDrive(hardwareMap, pose);
        this.drive = drive;

        this.telemetry = telemetry;
        this.limelightHardware2Axis = limelightHardware2Axis;
    }


    public void setVars(double x, double dxdt, double y, double dydt, double heading, double yaw){
        this.x = x;
        this.x2 = 60+x;//60 bc centered around the april tag not corner
        this.x3 = 60+x+cos(heading)*11;
        this.dxdt = dxdt;
        this.y = y;
        if(isBlue){
            this.y2 = 60+y;
            this.y3 = 60+y+sin(heading)*11;
        }else{
            this.y2 = 60-y;
            this.y3 = 60-y-sin(heading)*11;
        }

        this.dydt = dydt;
        this.heading = heading;
        this.yaw = yaw;// yaw is posotive goes counterclockwise
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("x speed", dxdt);
        telemetry.addData("y speed", dydt);
        telemetry.addData("heading", heading);
        telemetry.addData("heading speed", yaw);
        telemetry.addData("distance", Math.sqrt(x2*x2+y2*y2));
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("x speed", dxdt);
        packet.put("y speed", dydt);
        packet.put("heading", heading);
        packet.put("heading speed", yaw);

        packet.put("x2", x2);
        packet.put("y2", y2);
        packet.put("distance",Math.sqrt(x2*x2+y2*y2));
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
                -gamepad.right_stick_x * drive.getSpeedMultiplierRotate()
        );
        //driveClass.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad.left_stick_x, gamepad.left_stick_y), gamepad.right_stick_x));
        //drive.driveControl(1);
        telemetry.addData("change in heading", thetadt());


        TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
            Drawing.drawRobot(c, driveClass.localizer.getPose());
            packet.put("change in heading", thetadt());
        dashboard.sendTelemetryPacket(packet);




        boolean skipDrive = false;

        TelemetryPacket packet2 = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet2.fieldOverlay());
            if (action.run(packet2)) {
                newActions.add(action);
                skipDrive=true;
            }
        }
        runningActions = newActions;

        dashboard.sendTelemetryPacket(packet2);






        double targetHeading = Math.atan2(-y2,-x2);
        if (!isBlue){
            targetHeading = Math.atan2(y2,-x2);
        }
        double calcDif = calcDifference(targetHeading);

        if(!skipDrive){
            if (calcDif > Math.PI / 2 || calcDif < -Math.PI / 2) {
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);
            } else {
                //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,thetadt());
                if(false) {
                    drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,thetadt()+((calcDif/4)*((Math.PI/2)-Math.abs(calcDif))));
                }else{
                    drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);
                }
            }
        }


    }
    public void autoAlign(){
        isAligning=true;

        double targetHeading = Math.atan2(-y2,-x2);
        if (!isBlue){
            targetHeading = Math.atan2(y2,-x2);
        }
        double calcDif = calcDifference(targetHeading);
        drive.arcadeDriveSpeedControl2(0, 0, 0, calcDif/2);
        driveClass.updatePoseEstimate();
        runningActions.add(driveClass.actionBuilder(driveClass.localizer.getPose())
                .turn(Math.toRadians(calcDif))
                .build()

        );
    }
    public void endAutoAlign(){
        isAligning=false;

    }
    public void updateCamera(){
        limelightHardware2Axis.setServoAngles(0, Math.toDegrees(Math.atan(18/Math.sqrt(x3*x3+y3*y3))) );//22 should be hight difference of the camera and the april tags on the goals
        //can change to be x only and not pythagors theorem if camera is supposed to look at both april tags
    };

    private double thetadt (){
        if(isBlue){
            return (y2*dxdt-x2*dydt)/(x2*x2+y2*y2);
        }else{
            return -(y2*dxdt-x2*dydt)/(x2*x2+y2*y2);
        }

    }
    private double distdt(){
        if(isBlue){
            return x2+y2/Math.sqrt(x2*x2+y2*y2);
        }
        else{
            return x2+y2/Math.sqrt(x2*x2+y2*y2);
        }
    }
    private double calcDifference(double targetHeading){
        double difference = heading - targetHeading;

        // Normalize to [-π, π] range
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference < -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    };
    public void reset(boolean reset){

        TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
        Pose2D botPose = limelightHardware2Axis.getPos(c);
        if(botPose!=null&&reset){
            driveClass.localizer.setPose(new Pose2d(new Vector2d(botPose.getX(DistanceUnit.INCH),botPose.getY(DistanceUnit.INCH)), botPose.getHeading(AngleUnit.RADIANS)));
        }
        dashboard.sendTelemetryPacket(packet);
    }




}
