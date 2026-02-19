package org.firstinspires.ftc.teamcode.autos;

import static java.util.Arrays.asList;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Intake_Incomplete;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;

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

    public static Motif colorsToMotif(Intake_Incomplete.BallColor[] colors){

        if (colors[0] == Intake_Incomplete.BallColor.GREEN &&
                colors[1] == Intake_Incomplete.BallColor.PURPLE &&
                colors[2] == Intake_Incomplete.BallColor.PURPLE){
            return Motif.GPP;
        } else if (colors[0] == Intake_Incomplete.BallColor.PURPLE &&
                colors[1] == Intake_Incomplete.BallColor.GREEN &&
                colors[2] == Intake_Incomplete.BallColor.PURPLE){
            return Motif.PGP;
        } else if (colors[0] == Intake_Incomplete.BallColor.PURPLE &&
                colors[1] == Intake_Incomplete.BallColor.PURPLE &&
                colors[2] == Intake_Incomplete.BallColor.GREEN){
            return Motif.PPG;
        }

        return Motif.INVALID;
    }

    public static class shootKickingColor implements Action{
        private final RobotHardware robot;
        private final double distThreshold;
        private final Motif targetMotif;

        public shootKickingColor(RobotHardware hardware, double distThreshold, Motif TargetMotif){
            this.robot = hardware;
            this.distThreshold = distThreshold;
            this.targetMotif = TargetMotif;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){

            Intake_Incomplete.BallColor color1 = Color(robot, "left", distThreshold, Intake_Incomplete.BallColor.PURPLE);
            Intake_Incomplete.BallColor color2 = Color(robot, "middle", distThreshold, Intake_Incomplete.BallColor.PURPLE);
            Intake_Incomplete.BallColor color3 = Color(robot, "right", distThreshold, Intake_Incomplete.BallColor.PURPLE);

            Intake_Incomplete.BallColor[] colors = {color1, color2, color3};

            Motif IntookMotif = colorsToMotif(colors);

            int[] sequence = fireAutoKickerSeq(targetMotif, IntookMotif);

            ElapsedTime timer = new ElapsedTime();

            KickerKickInternal(robot, sequence[0]);
            while(timer.seconds() < 0.4){}
            KickerKickInternal(robot, sequence[1]);
            while(timer.seconds() < 0.8){}
            KickerKickInternal(robot, sequence[2]);

            return false;
        }
    }

    public static Intake_Incomplete.BallColor Color(RobotHardware hardware, String sensor, double distanceThreshold, Intake_Incomplete.BallColor previousColor){
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

    private static void KickerKickInternal(RobotHardware robot, int KickerPos){
        robot.kickers.fireKickerAuto(KickerPos);
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

    private static int[] fireAutoKickerSeq(Motif targetMotif, Motif intakeMotif)
    {
        if(targetMotif == null || targetMotif == Motif.INVALID){targetMotif = Motif.GPP;}
        if(intakeMotif == null || intakeMotif == Motif.INVALID){intakeMotif = Motif.GPP;}

        char[] targetMotifSeq = targetMotif.toString().toCharArray();
        char[] intakeMotifSeq = intakeMotif.toString().toCharArray();
        int[] seqArr = new int[3];
        int count = 0;
        for (char c : targetMotifSeq) {
            int pos = findChar(intakeMotifSeq, c);
            if (pos>-1 && pos<3) {
                seqArr[count]=pos;
                count++;
                intakeMotifSeq[pos] = 'x';
            }
        }

        return seqArr;
    }

    private static int findChar(char[] arr, char target) {
        if (arr == null) {
            return -1; // handle null array
        }

        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == target) {
                return i; // found, return index
            }
        }

        return -1; // not found
    }

}