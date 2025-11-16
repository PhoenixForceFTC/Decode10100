package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.autos.AutoActions;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;


@Autonomous(name="Auto Back Red")
public class Auto_Back_Red extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        _TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();
        Pose2d _beginPos = new Pose2d(60, -24, Math.PI);



        waitForStart();

        int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);
        int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);

        TrajectoryActionBuilder trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 0)) // speed is placeholder
                .stopAndAdd(new AutoActions.IntakeRun(_robot))

                // move to shooting area

                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                /*// shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[2]))
                .waitSeconds(2)

                // moves to first spike to intake artifacts, then moves back
                .splineTo(new Vector2d(-12, -48), -(Math.PI) / 4, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 15;
                    }
                })
                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                // kicks artifacts from first spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[2]))
                .waitSeconds(2)

                // moves to second spike to intake artifacts, then moves back
                .splineTo(new Vector2d(12, -48), -(Math.PI)/4)
                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                // kicks artifacts from second spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[2]))*/
                ;

        Actions.runBlocking(trajectoryActionBuilder.build());

        /*Actions.runBlocking(
                _robot.driveRR.mecanumDrive.actionBuilder(new Pose2d(-72, -24,0))
                        // starts intake and shooter
                        .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 0)) // speed is placeholder
                        .stopAndAdd(new AutoActions.IntakeRun(_robot))

                        // move to shooting area
                        .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                        // shoots the preloaded artifacts
                        .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[0]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[1]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[2]))
                        .waitSeconds(2)

                        // moves to first spike to intake artifacts, then moves back
                        .splineTo(new Vector2d(-12, -48), -(Math.PI)/4)
                        .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                        // kicks artifacts from first spike
                        .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[0]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[1]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[2]))
                        .waitSeconds(2)

                        // moves to second spike to intake artifacts, then moves back
                        .splineTo(new Vector2d(12, -48), -(Math.PI)/4)
                        .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                        // kicks artifacts from second spike
                        .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[0]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[1]))
                        .waitSeconds(0.5)
                        .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[2]))
                        .build());*/
        /*
        * Procedure (not pseudocode)
        * -Robot starts at back
        * -Scans obelisk for reading the motif
        * -moves to shooting area, faces goal
        * -Shoots
        * -Moves through back spike mark to intake artifacts
        * -moves back to shooting area
        * -shoots
        * -moves to loading zone, intakes artifacts
        * -moves back to shooting area
        * -shoots
        * */

    }

    private int[] fireAutoKickerSeq(LimelightHardware2Axis.Motif targetMotif, LimelightHardware2Axis.Motif intakeMotif)
    {
        char[] targetMotifSeq = targetMotif.toString().toCharArray();
        char[] intakeMotifSeq = intakeMotif.toString().toCharArray();
        int[] seqArr = new int[3];
        int count = 0;
        for (char c : targetMotifSeq) {
            int pos = findChar(intakeMotifSeq, c);
            if (pos>-1 && pos<3) {
                seqArr[count++]=pos;
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

    private class SlowVelConstriant {
    }
}
