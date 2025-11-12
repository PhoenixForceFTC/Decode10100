package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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




}