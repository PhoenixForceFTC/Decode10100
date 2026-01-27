package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;

import java.util.Arrays;

@Disabled
@Autonomous(name="Auton Load Blue")
public class Auto_Load_Blue extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;

    //slow down bot temporarily
    VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25.0),
            new AngularVelConstraint(Math.PI / 2)
    ));

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        _robot.limelightHardware2Axis.setServos(0.5,0.5);
        _robot.limelightHardware2Axis.servos();
        //_TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

        Pose2d _beginPos = new Pose2d(67, -15.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, _beginPos);


        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);

        while (!isStarted()) {
            _robot.limelightHardware2Axis.loop();
            if(_robot.limelightHardware2Axis.fiducialResultsContain(21)){
                _TargetMotif = LimelightHardware2Axis.Motif.GPP;
            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(22)){
                _TargetMotif = LimelightHardware2Axis.Motif.PGP;
            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(23)){
                _TargetMotif = LimelightHardware2Axis.Motif.PPG;
            }else{
                _TargetMotif = LimelightHardware2Axis.Motif.GPP; // prevent errors
            }
            telemetry.addData("target motif", _TargetMotif.toString());
            telemetry.update();
        }
        _robot.limelightHardware2Axis.servos();

        int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Towards opposite alliance loading zone
        int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);
        // Middle spike
        int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);
        // Towards goal
        int[] thirdSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);

        TrajectoryActionBuilder trajectoryActionBuilder1 = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2350)) // speed is placeholder
                .stopAndAdd(new AutoActions.IntakeRun(_robot))
                .waitSeconds(3)

                // move to shooting area

                .strafeTo(new Vector2d(-7, -7))
                .turn(Math.PI/4)
                .waitSeconds(3)

                // shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[2]))
                .waitSeconds(0.5)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))

                // in case auto is not able to be finished, we move to the side
                //.strafeToSplineHeading(new Vector2d(-60, -24), -Math.PI/2)

                // moves to first spike to intake artifacts, then moves back
                // TODO: make robot go 50% speed
                .strafeToSplineHeading(new Vector2d(36, 0), -Math.PI/4);
                /*.strafeToSplineHeading(new Vector2d(36, -48), -(Math.PI)/4, slowVelConstraint)
                .strafeToSplineHeading(new Vector2d(24, -24), -(3*Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-7, -7), -(3*Math.PI)/4)

                // kicks artifacts from first spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[2]))
                .waitSeconds(2)*/

                /*.stopAndAdd(new AutoActions.KickerUnkick(_robot, 0))
                .stopAndAdd(new AutoActions.KickerUnkick(_robot, 1))
                .stopAndAdd(new AutoActions.KickerUnkick(_robot, 2))

                //pickup from second spike
                .strafeToSplineHeading(new Vector2d(2.5, -27.5), -(Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(12, -48), -(Math.PI)/4, slowVelConstraint)
                .strafeToSplineHeading(new Vector2d(-7, -7), -(3*Math.PI)/4)

                //kicks artifacts from second spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[2]))
                .waitSeconds(2)


                .strafeToSplineHeading(new Vector2d(0, -24), -(3*Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-12, -48), -(3*Math.PI)/4, slowVelConstraint)
                .strafeToSplineHeading(new Vector2d(-7, -7), -(3*Math.PI)/4)

                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[2]))
                .waitSeconds(2)*/

        //TrajectoryActionBuilder TrajectoryActionBuilder2 = drive.actionBuilder(new Pose2d(-12, -12, -(3*Math.PI)4/))
        //        .splineTo(new Vector2d(24, -24), -(Math.PI)/4);

        Actions.runBlocking(trajectoryActionBuilder1.build());

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

    // intake motif is left to right kicker
    private int[] fireAutoKickerSeq(LimelightHardware2Axis.Motif targetMotif, LimelightHardware2Axis.Motif intakeMotif)
    {
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

    //private class SlowVelConstriant {
    //}
}
