package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.hardware.MotifKicking;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;


@Autonomous(name="8 Auto Red Close Dummy")
public class Auto_Red_Close_Dummy extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;
    private AutoActions.Motif _TargetMotifAction;

    //slow down bot temporarily
    VelConstraint fastVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(100),
            new AngularVelConstraint(2*Math.PI)
    ));

    ProfileAccelConstraint fastAccelConstraint = new ProfileAccelConstraint(-70, 100);
/*    VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25.0),
            new AngularVelConstraint(Math.PI / 2)
    ));*/

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        _robot.limelightHardware2Axis.setServos(0.4,0.5);
        _robot.limelightHardware2Axis.servos();
        _TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();  // just hardcoded to gpp

        Pose2d _beginPos = new Pose2d(39.5, -59.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, _beginPos);


        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);
        _robot.limelightHardware2Axis.servos();
        while (!isStarted()) {
            _robot.limelightHardware2Axis.loop();

            if(_robot.limelightHardware2Axis.fiducialResultsContain(21)){
                _TargetMotifAction = AutoActions.Motif.GPP;
                _TargetMotif = LimelightHardware2Axis.Motif.GPP;

            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(22)){
                _TargetMotifAction = AutoActions.Motif.PGP;
                _TargetMotif = LimelightHardware2Axis.Motif.PGP;
            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(23)){
                _TargetMotifAction = AutoActions.Motif.PPG;
                _TargetMotif = LimelightHardware2Axis.Motif.PPG;
            }else{
                _TargetMotifAction = AutoActions.Motif.GPP; // prevent errors
                _TargetMotif = LimelightHardware2Axis.Motif.GPP;
            }
            telemetry.addData("target motif", _TargetMotifAction.toString());
            telemetry.update();
            MotifKicking.updateMotif(_TargetMotifAction);
        }
        //_TargetMotif = LimelightHardware2Axis.Motif.GPP;
        telemetry.addData("target motif", _TargetMotifAction.toString());
        telemetry.update();
        _robot.limelightHardware2Axis.servos();

        int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        //Towards opposite alliance loading zone
        int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Middle spike
        int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);
        //Towards goal
        int[] thirdSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);

        TrajectoryActionBuilder trajectoryActionBuilder1 = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                //.stopAndAdd(new AutoActions.IntakeRun(_robot))
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2385)) // speed is placeholder
                //.stopAndAdd(new AutoActions.IntakeRunSlow(_robot))
                .stopAndAdd(new AutoActions.IntakeReverse(_robot))

                .waitSeconds(2.5)

                // move to shooting area

                // move to shooting area
                .strafeToSplineHeading(new Vector2d(28, -30), Math.toRadians(-61), fastVelConstraint, fastAccelConstraint)

                // shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerTripleKick(_robot))
                .waitSeconds(0.5)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .strafeToSplineHeading(new Vector2d(45, -30), Math.toRadians(-61), fastVelConstraint, fastAccelConstraint);

                /*
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2200))
                .stopAndAdd(new AutoActions.IntakeRunSlow(_robot))

                //go to first spike
                .strafeToSplineHeading(new Vector2d(8, -20), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(8, -60), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                //.turn((3*Math.PI)/4)
                //.strafeToSplineHeading(new Vector2d(-6, -58), -Math.PI/6, fastVelConstraint)
                //.strafeToSplineHeading(new Vector2d(-4, -60), Math.PI/6, fastVelConstraint)
                .stopAndAdd(new AutoActions.IntakeReverse(_robot))


//                //clear gate
                .strafeToSplineHeading(new Vector2d(0, -45), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(0, -57), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
//                //.turn(Math.PI/3)
//                //.strafeToSplineHeading(new Vector2d(0, -68), 0, fastVelConstraint)
//
//               //back to shooting zone
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(0, -32), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(43, -32), Math.toRadians(-45), fastVelConstraint, fastAccelConstraint)
//
//                // shoots the first spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.5)
//
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .waitSeconds(0.1)
                .stopAndAdd(new AutoActions.IntakeRunSlow(_robot))

//
//                //second spike
                .strafeToSplineHeading(new Vector2d(-20, -20), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-20, -68), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .stopAndAdd(new AutoActions.IntakeReverse(_robot))

                //back to shooting zone
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-20, -32), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(43, -32), Math.toRadians(-41), fastVelConstraint, fastAccelConstraint)
//
//                // shoots the second spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(1)
//

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .stopAndAdd(new AutoActions.IntakeRunSlow(_robot))


                //third spike
                .strafeToSplineHeading(new Vector2d(-49, -19), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-49, -67), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)

                .strafeToSplineHeading(new Vector2d(-20, -32), Math.toRadians(-90), fastVelConstraint, fastAccelConstraint)

                .stopAndAdd(new AutoActions.IntakeReverse(_robot))

                .strafeToSplineHeading(new Vector2d(23, -32), Math.toRadians(-41), fastVelConstraint, fastAccelConstraint);


                //back to shooting zone
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-20, 32), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(43, 32), Math.toRadians(33), fastVelConstraint, fastAccelConstraint)

                // shoots the third spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(1)
//
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))

                .strafeToSplineHeading(new Vector2d(60, 32), Math.toRadians(33), fastVelConstraint, fastAccelConstraint);
//
//                //third spike
//                .strafeToSplineHeading(new Vector2d(-55, 20), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
//                .strafeToSplineHeading(new Vector2d(-55, 68), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
//
//                //back to shooting
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(-36, 30), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
//                .strafeToSplineHeading(new Vector2d(54, 30), Math.toRadians(60), fastVelConstraint, fastAccelConstraint)
//
//                // shoots the third spike
//                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
//                .waitSeconds(0.5)
//
//                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
//
//                //loading zone
//                .strafeToSplineHeading(new Vector2d(-63.5, 30), Math.toRadians(0), fastVelConstraint, fastAccelConstraint)
//                .strafeToSplineHeading(new Vector2d(-63.5, 70.5), Math.toRadians(0), fastVelConstraint, fastAccelConstraint);

                  */


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

    //private class SlowVelConstraint {
    //}
}