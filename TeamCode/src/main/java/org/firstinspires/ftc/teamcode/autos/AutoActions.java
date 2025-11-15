package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class AutoActions {
    /// example classes

//    public static class WaitServoAction implements Action {
//        Servo servo;
//        double position;
//        ElapsedTime timer;
//
//        public WaitServoAction(Servo s, double p) {
//            this.servo = s;
//            this.position = p;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (timer == null) {
//                timer = new ElapsedTime();
//                servo.setPosition(position);
//            }
//            return timer.seconds() < 3;
//        }
//    }
//
//    // --- New Arm Action Classes ---
//
//    // Action to open the claw
//    public static class ArmClawOpenAction implements Action {
//        private final RobotHardware robot;
//
//        public ArmClawOpenAction(RobotHardware robot) {
//            this.robot = robot;
//            robot.arm._telemetry.addData("Initialize Arm", "-----------");
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            robot.arm.armClawOpen();
//            return false;
//        }
//    }
    public static class KickerLKick implements Action{
        private final RobotHardware robot;
        public KickerLKick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("left kicker flipped",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedL(true);
            return false;
        }
    }
    public static class KickerLUnkick implements Action{
        private final RobotHardware robot;
        public KickerLUnkick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("left kicker reset",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedL(false);
            return false;
        }
    }
    public static class KickerMKick implements Action{
        private final RobotHardware robot;
        public KickerMKick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("middle kicker flipped",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedM(true);
            return false;
        }
    }
    public static class KickerMUnkick implements Action{
        private final RobotHardware robot;
        public KickerMUnkick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("middle kicker reset",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedM(false);
            return false;
        }
    }
    public static class KickerRKick implements Action{
        private final RobotHardware robot;
        public KickerRKick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("right kicker flipped",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedR(true);
            return false;
        }
    }
    public static class KickerRUnkick implements Action{
        private final RobotHardware robot;
        public KickerRUnkick(RobotHardware robot){
            this.robot = robot;
            robot.kickers._telemetry.addData("right kicker reset",1);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.kickers.setKickedR(false);
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
            robot.intake.forward();
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