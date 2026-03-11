package org.firstinspires.ftc.teamcode.utils;
import com.acmerobotics.roadrunner.Pose2d;

public class Location {
    public static double x = -1;
    public static double y = -1;
    public static double heading = -1;
    public static Pose2d pose = new Pose2d(x,y,heading);

    public Location(){

    }
    public void setVars(double x,double y,double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.pose = new Pose2d(x,y,heading);
    }
    public static void SetPose(Pose2d pose){
        Location.pose = pose;
        Location.x = pose.position.x;
        Location.y = pose.position.y;
        Location.heading = pose.heading.toDouble();
    }
    public static Pose2d GetPose(){
        return new Pose2d(x,y,heading);
    }


}
