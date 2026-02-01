package org.firstinspires.ftc.teamcode.autos;

import static java.util.Arrays.asList;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Intake_Incomplete;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public class AutoActions {

    public enum Motif {
        PPG,
        PGP,
        GPP,
        INVALID;

    }

    public Motif colorsToMotif(Intake_Incomplete.BallColor[] colors){

        List<Intake_Incomplete.BallColor> colorsList = Arrays.asList(colors);

        //check if item is valid
        if(colorsList.contains(Intake_Incomplete.BallColor.PURPLE)){
            colorsList.remove(Intake_Incomplete.BallColor.PURPLE);
            if(!colorsList.contains(Intake_Incomplete.BallColor.PURPLE)){return Motif.INVALID;}
            if(!colorsList.contains(Intake_Incomplete.BallColor.GREEN)){return Motif.INVALID;}
        }

        if(colors[0] == Intake_Incomplete.BallColor.GREEN){
            return Motif.GPP;
        } else if (colors[1] == Intake_Incomplete.BallColor.GREEN){
            return Motif.PGP;
        } else if (colors[2] == Intake_Incomplete.BallColor.GREEN){
            return Motif.PPG;
        }

        return Motif.INVALID;
    }

    public static class shootKickingColor implements Action{
        private final RobotHardware robot;
        private final double distThreshold;

        public shootKickingColor(RobotHardware hardware, double distThreshold){
            this.robot = hardware;
            this.distThreshold = distThreshold;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){

            return false;
        }
    }

    public Intake_Incomplete.BallColor Color(RobotHardware hardware, String sensor, double distanceThreshold, Intake_Incomplete.BallColor previousColor){
        RobotHardware robot;
        ColorSensor colorSensor;
        DistanceSensor distanceSensor;
        double distThreshold;

        robot = hardware;
        distThreshold = distanceThreshold;
        if(sensor.equals("left")){
            colorSensor = robot.colorSensorLeft;
        }else if(sensor.equals("middle")){
            colorSensor  = robot.colorSensorMiddle;
        }else if(sensor.equals("right")){
            colorSensor = robot.colorSensorRight;
        }else{
            colorSensor = robot.colorSensorLeft;
        }

        distanceSensor = (DistanceSensor) colorSensor;
        

        return robot.intake.detectBallSticky(colorSensor, distanceSensor, distanceThreshold, Intake_Incomplete.BallColor.PURPLE);
    }

    public static class KickerKick implements Action{
        private final RobotHardware robot;
        private int pos;
        public KickerKick(RobotHardware robot, int kickerPos){
            this.robot = robot;
            //robot.kickers._telemetry.addData("left kicker flipped",1);
            this.pos = kickerPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.fireKickerAuto(this.pos);
            return false;
        }
    }

    public static class KickerTripleKick implements Action{
        private final RobotHardware robot;
        public KickerTripleKick(RobotHardware robot){
            this.robot = robot;
            //robot.kickers._telemetry.addData("left kicker flipped",1);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.fireKickerAuto(3);
            return false;
        }
    }

    public static class KickerUnkick implements Action{
        private final RobotHardware robot;
        public KickerUnkick(RobotHardware robot){
            this.robot = robot;
            //robot.kickers._telemetry.addData("left kicker flipped",1);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.retractKickerAuto(0);
            robot.kickers.retractKickerAuto(1);
            robot.kickers.retractKickerAuto(2);
            return false;
        }
    }

    public static class IntakeRun implements Action{
        private final RobotHardware robot;
        public IntakeRun(RobotHardware robot){
            this.robot = robot;
            robot.intake._telemetry.addData("Intake running",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.backward();
            return false;
        }
    }

    public static class IntakeRunSlow implements Action{
        private final RobotHardware robot;

        public IntakeRunSlow(RobotHardware robot){
            this.robot = robot;
            robot.intake._telemetry.addData("Intake running",1);

        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            robot.intake.backward_slow();
            return false;
        }

    }
    public static class IntakeStop implements Action{
        private final RobotHardware robot;
        public IntakeStop(RobotHardware robot){
            this.robot = robot;
            robot.intake._telemetry.addData("Intake running",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.stop();
            return false;
        }
    }
    public static class IntakeReverse implements Action{
        private final RobotHardware robot;
        public IntakeReverse(RobotHardware robot){
            this.robot = robot;
            robot.intake._telemetry.addData("Intake running",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.backward();
            return false;
        }
    }
    public static class SetShooterSpeed implements Action{
        private final RobotHardware robot;
        private final int speed;
        public SetShooterSpeed(RobotHardware robot,int speed){
            this.robot = robot;
            this.speed = speed;
            robot.shooter._telemetry.addData("shooter speed speeding",speed);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.shooter.shoot(speed);
            return false;
        }
    }


}