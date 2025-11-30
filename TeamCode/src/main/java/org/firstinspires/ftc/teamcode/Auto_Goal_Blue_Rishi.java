package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autos.AutoActions;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;


@Autonomous(name="Auton Goal Blue Rishi")
public class Auto_Goal_Blue_Rishi extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif = null;



    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        //_TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

        Pose2d _beginPos = new Pose2d(-39.5, -61.5, Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap,_beginPos);

        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);
        _robot.limelightHardware2Axis.setServos(0.4, 0.5);
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
            }
            if(_TargetMotif!=null){
                telemetry.addData("target motif", _TargetMotif.toString());
            }else{
                telemetry.addLine("no target motif detected");
            }

            telemetry.update();
        }
        if(_TargetMotif==null){
            _TargetMotif=LimelightHardware2Axis.Motif.GPP;
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

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2350)) // speed is placeholder
                .stopAndAdd(new AutoActions.IntakeRun(_robot))

                // move to shooting area


                .strafeToSplineHeading(new Vector2d(-7, -30), -3*Math.PI/4)

                //shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, initialKickingOrder[2]))
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(-39, -12), -(3*Math.PI)/4);

                /*
                // moves to third spike to intake artifacts, then moves back
                //.strafeToSplineHeading(new Vector2d(0, -36), -(3*Math.PI)/4)
                .turn((Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-6, -42), -(Math.PI)/2,GetAccelConstraint(0.3))
                .strafeToSplineHeading(new Vector2d(-18, -45), -(Math.PI)/2,GetAccelConstraint(0.2))
                .strafeToSplineHeading(new Vector2d(-12, -12), -(3*Math.PI)/4)

                // kicks artifacts from third spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, secondSpikeKickingOrder[2]))
                .waitSeconds(2)

                // moves to second spike to intake artifacts, then moves back
                .strafeToSplineHeading(new Vector2d(12, -48), -(Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-12, -12), -(3*Math.PI)/4)

                // kicks artifacts from second spike
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[0]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[1]))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerKick(_robot, thirdSpikeKickingOrder[2]));*/

        Actions.runBlocking(trajectoryActionBuilder.build());

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

    private class SlowVelConstriant {
    }
    private TranslationalVelConstraint GetTranslationalVelConstraint(double poewr){
        return new TranslationalVelConstraint(50*poewr);
    }
    private VelConstraint GetAccelConstraint(double poewr){
        VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint((50*poewr)),
                        new AngularVelConstraint(Math.PI/2)
                ));
        return slowVelConstraint;
    }
}
