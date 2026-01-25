package org.firstinspires.ftc.teamcode.utils;
import com.acmerobotics.roadrunner.Pose2d;

public class Location {
    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;
    public static Pose2d pose = new Pose2d(x,y,heading);

    public Location(){

    }
    public void setVars(double x,double y,double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.pose = new Pose2d(x,y,heading);
    }
    public void setPose(Pose2d pose){
        this.pose = pose;
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }


}
