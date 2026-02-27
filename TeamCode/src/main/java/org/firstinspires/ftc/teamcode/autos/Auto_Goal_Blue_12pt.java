package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.LimelightHardware2Axis;
import org.firstinspires.ftc.teamcode.hardware.MotifKicking;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;


@Autonomous(name="Auton Goal Blue 12pt")
public class Auto_Goal_Blue_12pt extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;
    private AutoActions.Motif _TargetMotifAction;

    //slow down bot temporarily
    VelConstraint fastVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(100),
            new AngularVelConstraint(2*Math.PI)
    ));

    ProfileAccelConstraint fastAccelConstraint = new ProfileAccelConstraint(-70, 100);
    VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25.0),
            new AngularVelConstraint(Math.PI / 2)
    ));

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        _robot.limelightHardware2Axis.setServos(0.4,0.5);
        _robot.limelightHardware2Axis.servos();
        _TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

        Pose2d _beginPos = new Pose2d(-39.5, -59.5, Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, _beginPos);


        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);
        _robot.limelightHardware2Axis.servos();
        while (!isStarted()) {
            _robot.limelightHardware2Axis.loop();
            if(_robot.limelightHardware2Axis.fiducialResultsContain(21)){
                _TargetMotifAction = AutoActions.Motif.GPP;
            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(22)){
                _TargetMotifAction = AutoActions.Motif.PGP;
            }
            else if(_robot.limelightHardware2Axis.fiducialResultsContain(23)){
                _TargetMotifAction = AutoActions.Motif.PPG;
            }else{
                _TargetMotifAction = AutoActions.Motif.GPP; // prevent errors
            }
            telemetry.addData("target motif", _TargetMotif.toString());
            telemetry.update();
            MotifKicking.updateMotif(_TargetMotifAction);
        }
        _TargetMotif = LimelightHardware2Axis.Motif.GPP;
        telemetry.addData("target motif", _TargetMotif.toString());
        telemetry.update();
        _robot.limelightHardware2Axis.servos();

        //int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Towards opposite alliance loading zone
        //int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Middle spike
        //int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);
        // Towards goal
        //int[] thirdSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);

        TrajectoryActionBuilder trajectoryActionBuilder1 = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.IntakeRun(_robot))
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2400)) // speed is placeholder
                .stopAndAdd(new AutoActions.IntakeRunSlow(_robot))
                .waitSeconds(2.5)

                // move to shooting area

                // move to shooting area
                .strafeToSplineHeading(new Vector2d(-28, -30), -(2*Math.PI)/3, fastVelConstraint, fastAccelConstraint)

                // shoots the preloaded artifacts
                .stopAndAdd(new AutoActions.KickerTripleKick(_robot))
                .waitSeconds(0.4)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2200))

                //go to first spike
                .strafeToSplineHeading(new Vector2d(-12, -20), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-12, -60), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                //.turn((3*Math.PI)/4)
                //.strafeToSplineHeading(new Vector2d(-6, -58), -Math.PI/6, fastVelConstraint)
                //.strafeToSplineHeading(new Vector2d(-4, -60), Math.PI/6, fastVelConstraint)

                //clear gate
                .strafeToSplineHeading(new Vector2d(0, -40), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(3, -60), -Math.PI/2,fastVelConstraint, fastAccelConstraint)
                //.turn(Math.PI/3)
                //.strafeToSplineHeading(new Vector2d(0, -68), 0, fastVelConstraint)

                //back to shooting zone
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-30, -32), -(5*Math.PI)/6, fastVelConstraint, fastAccelConstraint)

                // shoots the first spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.2)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))

                //second spike
                .strafeToSplineHeading(new Vector2d(18, -20), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(18, -72), -Math.PI/2, fastVelConstraint, fastAccelConstraint)

                //back to shooting
                .strafeToSplineHeading(new Vector2d(18, -40), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4,fastVelConstraint, fastAccelConstraint)



                // shoots the second spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.2)
                .waitSeconds(0.2)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))

                //third spike
                .strafeToSplineHeading(new Vector2d(44, -20), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(44, -72), -Math.PI/2, fastVelConstraint, fastAccelConstraint)

                //back to shooting
                .waitSeconds(2)
                .strafeToSplineHeading(new Vector2d(36, -40), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-30, -30), -(3*Math.PI)/4, fastVelConstraint, fastAccelConstraint)

                // shoots the third spike
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.2)

                .stopAndAdd(new AutoActions.KickerUnkick(_robot))

                //loading zone
                .strafeToSplineHeading(new Vector2d(63.5, -30), -Math.PI/2, fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(63.5, -70.5), -Math.PI/2, fastVelConstraint, fastAccelConstraint);

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
