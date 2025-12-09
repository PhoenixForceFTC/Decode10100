package org.firstinspires.ftc.teamcode.utils;

public class DataLog {
    private double x =-999;
    private double y =-999;
    private double heading =-999;

    private double camera_yaw =-999;
    private double camera_pitch =-999;

    private static enum Team{
        Red,
        Blue,
        Unkown
    }
    private Team color = Team.Unkown;
    private Boolean suceed = false;
    private double targetRpmPre=-999;
    private double targetRpmPost=-999;
    private double shooterRpmPre=-999;
    private double shooterRpmPost=-999;
    public static enum Shooter{
        Left,Middle,Right,All,Unkown
    }
    private Shooter shooter = Shooter.Unkown;

    public DataLog(double x,double y,double heading,double camera_yaw,double camera_pitch,Boolean suceed,double targetRpmPre,double targetRpmPost,double shooterRpmPre,double shooterRpmPost,Shooter shooter){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.camera_yaw = camera_yaw;
        this.camera_pitch = camera_pitch;
        this.suceed = suceed;
        this.targetRpmPre = targetRpmPre;
        this.targetRpmPost = targetRpmPost;
        this.shooterRpmPre = shooterRpmPre;
        this.shooterRpmPost = shooterRpmPost;
        this.shooter = shooter;
    }

    @Override
    public String toString(){
        String value = String.format("%.3f,%.3f,%3f,%3f,%3f,%s,%b,%3f,%3f,%3f,%3f,%s",x,y,heading,camera_yaw,camera_pitch,color.toString(),suceed,targetRpmPre,targetRpmPost,shooterRpmPre,shooterRpmPost,shooter.toString());
        return value;
    }




}
