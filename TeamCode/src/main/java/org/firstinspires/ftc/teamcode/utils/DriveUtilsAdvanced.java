package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Kickers;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

//TODO: important things to do
//TODO: make it only auto shoot the balls after align if in the shooting zone
//TODO: fix the camera
//TODO: try to use the dcmotorex on shooters to detect current
//TODO: only reset position if auto was the last thing run
//auto reverse intake
//lights flash red when shoot
//shooting while moving
//turn faster
@Config
public class DriveUtilsAdvanced {
    private List<Action> runningActions = new ArrayList<>();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean isBlue = false;
    public boolean isAligning = false;
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

    private double tagXposition = 0.0;
    private int targetTagId = 0;

    private RobotHardware _robot;

    private Telemetry telemetry;

    public Drive drive;

    private MecanumDrive driveClass;

    private LimelightHardware2Axis limelightHardware2Axis;

    double adjustmentDegrees(){//higher value will turn it more clockwise
        if(isBlue){
            return -8;
        }else{
            return -2;
        }
     };
    public DriveUtilsAdvanced(HardwareMap hardwareMap, Pose2d pose, Drive drive,LimelightHardware2Axis limelightHardware2Axis,
                              Telemetry telemetry,Boolean isBlue, RobotHardware robotHardware){
        driveClass = new MecanumDrive(hardwareMap, pose);
        isAligning=false;
        this.drive = drive;
        this.isBlue=isBlue;
        this.telemetry = telemetry;
        this.limelightHardware2Axis = limelightHardware2Axis;
        this._robot = robotHardware;
        if (isBlue)
        {
            this.targetTagId = 20;
        }
        else
        {
            this.targetTagId = 24;
        }
        telemetry.addData("location string in driveUtils", Location.GetPose());
        telemetry.addData("Class Hash in driveUtils", Location.class.hashCode());
        telemetry.addData("rr pinpoint localizer", driveClass.localizer.getPose().toString());

    }


    private void setVars(double x, double dxdt, double y, double dydt, double heading, double yaw){
        this.x = x;
        this.x2 = 60+x;//60 bc centered around the april tag not corner
        this.x3 = 60+x+cos(heading)*11;
        this.x4 = 72+x;
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
        telemetry.addData("target heading degrees",Math.toDegrees(getTargetHeading(y4-14.55098425, x4-11.82122047)));
        telemetry.addData("calcDiff degrees", Math.toDegrees(calcDifference(getTargetHeading(y4-14.55098425, x4-11.82122047))));
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
        packet.put("target heading degrees",Math.toDegrees(getTargetHeading(y4-14.55098425, x4-11.82122047)));
        packet.put("calcDiff degrees", Math.toDegrees(calcDifference(getTargetHeading(y4-14.55098425, x4-11.82122047))));
        dashboard.sendTelemetryPacket(packet);
    }
    private void setVarsAdvanced(double x, double y, double heading,double axial, double lateral, double yaw){
        setVars(x,lateral*sin(heading)+axial*cos(heading),y,axial*sin(heading)-lateral*cos(heading),heading,yaw);

    };

    // returns true if trying to auto align and the auto align is finished
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

        TelemetryPacket packet2 = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        boolean skipDrive = false;
        if(runningActions.isEmpty()){
            telemetry.addLine("RR is not running");
            packet2.addLine("RR is not running");
        }
        else
        {
            for (Action action : runningActions) {
                telemetry.addLine("RR is running");
                packet2.addLine("RR is running");
                action.preview(packet2.fieldOverlay());
                if (action.run(packet2)) {   // this is a blocking call but may not finish fully
                    newActions.add(action);
                    skipDrive=true;
                }
            }
        }
        runningActions = newActions;

        dashboard.sendTelemetryPacket(packet2);

        double targetHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
        double calcDif = calcDifference(targetHeading);//calc diff uses road


        calcDif+=Math.toRadians(adjustmentDegrees());




////if raodrunners drive class is not running then we will run our code and if we are within the correct range of the target heading
//// we will power our motors with speed that is currently proportionaly with the heading angle we have to change*//
//        //we will calculate the heading angle we have to change at first by using all data collected by deadwheels in our localizer
//        //and then we will use only heading data in combination with camera data to calculate the difference
//        // skipDrive=true: robot still under RR control, got to wait
        if(!skipDrive){
            if (calcDif > Math.PI / 2 || calcDif < -Math.PI / 2) {
                // this is the default drive signal
                // yawImportant = 0 means no additional turn power
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);

                // todo: trying driveRR to see if that is faster, tried driveRR but was messing up auto align
                //_robot.driveRR.driveControl(1.0);

            }
            else
            {
                if(isAligning) {  //right trigger sets this and reset after shooting
                    telemetry.addData("Aligning:","Yes");
                    returnn = autoAlignViaLLandPower(gamepad,calcDif);  // return true if we are close enough and aligned.
                }
                else
                {
                    // _robot.driveRR.driveControl(1.0);  // maybe faster but autoaligning not working
                   drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, 0);
                }
            }
        }

        return returnn;


    }
//
    private boolean autoAlignViaLLandPower(Gamepad gamepad,double calcDif)
    {
//        if(Math.abs(calcDif)>Math.PI/8) {//this is
//            drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));
//            // todo: fix calcdif auto align first then comment out calcdiff auto align for the smaller angles and use camera
//            //auto aligns using roadrunner position do first
//        }
//        else
//        {
            double[] distBreakdown = limelightHardware2Axis.getDistanceBreakdown();
            //distBreakdown = null; // temp todo: can we just use
            double angleToTurnFromCamera = limelightHardware2Axis.getTxDegreesForId(this.targetTagId);

            calcDif += Math.toRadians(adjustmentDegrees());
            angleToTurnFromCamera+= adjustmentDegrees();
            if(Math.abs(angleToTurnFromCamera)< 1){ // within 2 degrees
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,0); // done turn off power
                if(dxdt<0.1&&dydt<0.1&&yaw<0.1) {
                    return true;
                }
            }
            if(/*distBreakdown == null */angleToTurnFromCamera > (double)120.0){
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
            }
            else
            {
                //todo: fix the camera auto aligning
                //chnge to do atan using trig and subtracting the heading from the loclizer

                //posibilty not usiing camera is better
                //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x,
                        Math.toRadians(angleToTurnFromCamera)*0.8);//-angleToTurnFromCamera bc posotive turn makes it turn clockwise in the method
                //drive.arcadeDriveSpeedControl2(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, thetadt() + (calcDif / 3));//it is only turning right and not left maybe
                // todo: temp uncomment line above and enable camera
                //auto aligns using roadrunner position do first disable once camera works

            }

//        }

        return false;
    }

    // call from OpMode loop, to see what calcDiff is currently
    public void printCalcDiff(){
        double targetHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
        double calcDif = calcDifference(targetHeading);

        telemetry.addData("Calc Turn Val:",Math.toDegrees(calcDif));
    }

    public void printXDegrees(){
        double angleToTurnFromCamera = limelightHardware2Axis.getTxDegreesForId(this.targetTagId);
        telemetry.addData("Target X Degrees:", angleToTurnFromCamera);
    }

    //  Use road runner first to get it to be able to see april tag
   public void autoAlign(){
        isAligning=true;

        double targetHeading = getTargetHeading(y4-14.55098425, x4-11.82122047);
        double calcDif = calcDifference(targetHeading);

        calcDif+=Math.toRadians(adjustmentDegrees());


        //drive.arcadeDriveSpeedControl2(0, 0, 0, calcDif/2);
        //drive.arcadeDriveSpeedControl2(0, 0, 0, 0);  -> anyway done when skipDrive is setup
        driveClass.updatePoseEstimate();  // only dashboard update
        if(runningActions.isEmpty()) {
            //todo: uncomment out once you get both calcdif and camera alignign working because those are always aligning and are more accurate
            runningActions.add(driveClass.actionBuilder(driveClass.localizer.getPose())
                    .turn(-calcDif)
                    .build()
            );
        }
    }

    private double getTargetHeading(double targetLocationY, double targetLocationX) {
        double targetHeading = Math.atan2(-targetLocationY, -targetLocationX);
        if (!isBlue) {
            targetHeading = Math.atan2(targetLocationY, -targetLocationX);
        }
        return targetHeading;
    }

    public double getDist(){
        return Math.sqrt(x2*x2+y2*y2);
        //1.01x+1630 for single shooter
    }

    // Must be called else robot control will be off
    public void endAutoAlign(){
        isAligning=false;
    }

    public void updateCameraPitch(){
        limelightHardware2Axis.setServoAngles(0, Math.toDegrees(Math.atan(18/Math.sqrt(x3*x3+y3*y3))) );//22 should be hight difference of the camera and the april tags on the goals
        telemetry.addData("pitch angle", Math.toDegrees(Math.atan(18/Math.sqrt(x3*x3+y3*y3))));
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

    // Resets position of localizer based on camera
    // currently this is called in the loop of the opmode every 20 times
    public void reset(boolean reset){

        TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
        Pose2D botPose = limelightHardware2Axis.getRobotPos(c); // is null when camera cannot tell position

        if(runningActions.isEmpty()){
            if(botPose!=null&&reset){
                if( Math.abs(botPose.getHeading(AngleUnit.RADIANS)-driveClass.localizer.getPose().heading.toDouble()) < Math.PI/16){
                    // TODO: why does it only reset if the error is above 4 inches
                    //TODO: fix it so camera always gets right position
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
