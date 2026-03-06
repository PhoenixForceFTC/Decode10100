package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Kickers;
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
    private double x4 = 72;
    private double dxdt = 0;
    private double y = 0;
    private double y2 = 60;
    private double y3 = 60;

    private double y4 = 72;
    private double dydt = 0;
    private double heading = 0;
    private double yaw = 0;

    private Telemetry telemetry;

    public Drive drive;

    private MecanumDrive driveClass;

    private LimelightHardware2Axis limelightHardware2Axis;
    public DriveUtilsAdvanced(HardwareMap hardwareMap, Pose2d pose, Drive drive,LimelightHardware2Axis limelightHardware2Axis, Telemetry telemetry,Boolean isBlue){
        driveClass = new MecanumDrive(hardwareMap, pose);
        isAligning=false;
        this.drive = drive;
        this.isBlue=isBlue;
        this.telemetry = telemetry;
        this.limelightHardware2Axis = limelightHardware2Axis;
    }


    public void setVars(double x, double dxdt, double y, double dydt, double heading, double yaw){
        this.x = x;
        this.x2 = 60+x;//60 bc centered around the april tag not corner
        this.x3 = 60+x+cos(heading)*11;
        this.x4= 72+x;
        this.dxdt = dxdt;
        this.y = y;
        if(isBlue){
            this.y2 = 60+y;
            this.y3 = 60+y+sin(heading)*11;
            this.y4 = 72+y;
        }else{
            this.y2 = 60-y;
            this.y3 = 60-y-sin(heading)*11;
            this.y4 = 72-y;
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
    public boolean driveMecanum(Gamepad gamepad, Kickers kickers){
        boolean returnn = false;

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
        if(runningActions.isEmpty()){
            telemetry.addLine("RR is not running");
            packet2.addLine("RR is not running");
        }
        for (Action action : runningActions) {
            telemetry.addLine("RR is running");
            packet2.addLine("RR is running");
            action.preview(packet2.fieldOverlay());
            if (action.run(packet2)) {
                newActions.add(action);
                skipDrive=true;
            }
        }
        runningActions = newActions;


        dashboard.sendTelemetryPacket(packet2);






        double targetHeading = Math.atan2(-y4,-x4);
        if (!isBlue){
            targetHeading = Math.atan2(y4,-x4);
        }
        double calcDif = calcDifference(targetHeading);//calc diff uses road

        if(!skipDrive){
            if (calcDif > Math.PI / 2 || calcDif < -Math.PI / 2) {
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);
            } else {
                //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,thetadt());
                if(isAligning) {
                    if(Math.abs(calcDif)>Math.PI/8) {//this is
                        drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                        // todo: fix calcdif auto align first then comment out calcdiff auto align for the smaller angles and use camera
                        //auto aligns using roadrunner position do first
                    }else{
                        double[] distBreakdown = limelightHardware2Axis.getDistanceBreakdown();
                        double angleToTurnFromCamera = 0;
                        if(distBreakdown == null ){
                            angleToTurnFromCamera =0;
                            drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                        }else{
                            //angleToTurnFromCamera = Math.atan2(distBreakdown[2],distBreakdown[1]);//we want to change to point towards the back corner not the aprilt ag
                            if(isBlue){
                                angleToTurnFromCamera = Math.atan2(distBreakdown[1] * Math.sin(heading) - distBreakdown[2] * Math.cos(heading) - 14.55098425, distBreakdown[2] * Math.cos(heading) + distBreakdown[1] * Math.sin(heading) + 11.82122047);
                            }
                            else {
                                angleToTurnFromCamera = Math.atan2(distBreakdown[1] * Math.sin(heading) - distBreakdown[2] * Math.cos(heading) + 14.55098425, distBreakdown[2] * Math.cos(heading) + distBreakdown[1] * Math.sin(heading) + 11.82122047);
                            }
                            angleToTurnFromCamera -= heading;//non roadrunner from camera
                            //todo: fix the camera auto aligning
                            telemetry.addData("angleToTurnFromCamera", angleToTurnFromCamera);
                            //chnge to do atan using trig and subtracting the heading from the loclizer
                            if(Math.abs(angleToTurnFromCamera)<Math.PI/16){
                                returnn =true;
                                //fix so it doesnt shoot with the wrong speeds and shoots using the three ball speeds or something
                            }
                            //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, -angleToTurnFromCamera);//-angleToTurnFromCamera bc posotive turn makes it turn clockwise in the method
                            drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                            // todo: temp uncomment line above and enable camera
                            //auto aligns using roadrunner position do first disable once camera works

                        }

                    }

                }else{
                    drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);
                }
            }
        }
        return returnn;


    }
    public void autoAlign(){
        isAligning=true;

        double targetHeading = Math.atan2(-y4,-x4);
        if (!isBlue){
            targetHeading = Math.atan2(y4,-x4);
        }
        double calcDif = calcDifference(targetHeading);
        drive.arcadeDriveSpeedControl2(0, 0, 0, calcDif/2);
        driveClass.updatePoseEstimate();
        if(runningActions.isEmpty()) {
            runningActions.add(driveClass.actionBuilder(driveClass.localizer.getPose())
//                    .turn(-calcDif)
                    .build()

            );
        }
    }
    public double getDist(){
        return Math.sqrt(x2*x2+y2*y2);
        //1.01x+1630 for single shooter
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
        if(runningActions.isEmpty()){

            if(botPose!=null&&reset){
                if( Math.abs(botPose.getHeading(AngleUnit.RADIANS)-driveClass.localizer.getPose().heading.toDouble()) < Math.PI/16){
                    if(Math.abs(botPose.getX(DistanceUnit.INCH)-driveClass.localizer.getPose().position.x)>4 || Math.abs(botPose.getY(DistanceUnit.INCH)-driveClass.localizer.getPose().position.y)>4){
                        driveClass.localizer.setPose(new Pose2d(new Vector2d(botPose.getX(DistanceUnit.INCH),botPose.getY(DistanceUnit.INCH)), driveClass.localizer.getPose().heading.toDouble()));

                    }
                }else{
                    driveClass.localizer.setPose(new Pose2d(new Vector2d(botPose.getX(DistanceUnit.INCH),botPose.getY(DistanceUnit.INCH)), botPose.getHeading(AngleUnit.RADIANS)));
                }
            }
        }
        Drawing.drawRobot(c, driveClass.localizer.getPose());
        dashboard.sendTelemetryPacket(packet);
    }




}
