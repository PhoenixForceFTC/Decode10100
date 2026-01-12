package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;


@Autonomous(name="Auton Goal Blue 12pt")
public class Auto_Goal_Blue_12pt extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;

    //slow down bot temporarily
    VelConstraint fastVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(80),
            new AngularVelConstraint(Math.PI)
    ));
    VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25.0),
            new AngularVelConstraint(Math.PI / 2)
    ));

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        //_robot.limelightHardware2Axis.setServos(0.4,0.6);
        //_robot.limelightHardware2Axis.servos();
        //_TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

        Pose2d _beginPos = new Pose2d(-39.5, -59.5, Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, _beginPos);


        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);
        _robot.limelightHardware2Axis.servos();
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
        _TargetMotif = LimelightHardware2Axis.Motif.GPP;
        telemetry.addData("target motif", _TargetMotif.toString());
        telemetry.update();
        _robot.limelightHardware2Axis.servos();

        int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Towards opposite alliance loading zone
        int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Middle spike
        int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);
        // Towards goal
        int[] thirdSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);

        TrajectoryActionBuilder trajectoryActionBuilder1 = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2350)) // speed is placeholder
                .stopAndAdd(new AutoActions.IntakeRunSlow(_robot))
                .waitSeconds(3)

                // move to shooting area

//                .strafeTo(new Vector2d(-7, -7))
//                .turn(Math.PI/4)
//                .waitSeconds(3)

                // move to shooting area
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4, fastVelConstraint)


                // shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[2]))
                .waitSeconds(0.5)

                //go to first spike
                .turn((3*Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-12, -64), 0, fastVelConstraint)
                .strafeToSplineHeading(new Vector2d(-16, -60), 0, fastVelConstraint)
                .strafeToSplineHeading(new Vector2d(-12, -64), 0, slowVelConstraint)

                //clear gate
                .strafeToSplineHeading(new Vector2d(0, -57.5), 0, fastVelConstraint)

                //back to shooting zone
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4, fastVelConstraint)

                // shoots the first spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, firstSpikeKickingOrder[2]))
                .waitSeconds(0.5)

                //second spike
                .strafeToSplineHeading(new Vector2d(12, -48), 0, fastVelConstraint)

                //back to shooting
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4,fastVelConstraint)

                // shoots the second spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[2]))
                .waitSeconds(0.5)

                //third spike
                .strafeToSplineHeading(new Vector2d(36, -48), 0, fastVelConstraint)

                //back to shooting
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4, fastVelConstraint)

                // shoots the second spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[2]))
                .waitSeconds(0.5)

                //loading zone
                .strafeToSplineHeading(new Vector2d(63.5, -30), -Math.PI/2, fastVelConstraint)
                .strafeToSplineHeading(new Vector2d(63.5, -70.5), -Math.PI/2, fastVelConstraint);

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
        if(targetMotif == null){targetMotif = LimelightHardware2Axis.Motif.GPP;}

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
