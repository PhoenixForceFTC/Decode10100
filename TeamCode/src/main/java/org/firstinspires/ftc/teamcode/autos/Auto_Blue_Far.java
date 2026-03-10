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
import org.firstinspires.ftc.teamcode.utils.Location;

import java.util.Arrays;


@Autonomous(name="Auto Blue Far")
public class Auto_Blue_Far extends LinearOpMode{
    RobotHardware _robot = new RobotHardware(this);
    private LimelightHardware2Axis.Motif _TargetMotif;

    private AutoActions.Motif _TargetMotifAction;

    VelConstraint fastVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(100),
            new AngularVelConstraint(2*Math.PI)
    ));

    ProfileAccelConstraint fastAccelConstraint = new ProfileAccelConstraint(-70, 100);

    @Override
    public void runOpMode() throws InterruptedException {
        //incomplete
        _robot.init(1); // Initialize robot parts
        //_TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

        Pose2d _beginPos = new Pose2d(-63, 12, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,_beginPos);

        //waitForStart();
        _robot.limelightHardware2Axis.setPipeline(1);
        _robot.limelightHardware2Axis.setServos(0.5, 0.5);
        _robot.limelightHardware2Axis.servos();
        _TargetMotif = _robot.limelightHardware2Axis.getObliskTagId();

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
            telemetry.addData("target motif", _TargetMotif.toString());
            telemetry.addData("location string in teleopState", Location.pose.toString());
            telemetry.addData("Class Hash in teleopState", Location.class.hashCode());
            telemetry.update();
            MotifKicking.updateMotif(_TargetMotifAction);
        }
       telemetry.addData("target motif", _TargetMotif.toString());
        telemetry.addData("location string in teleopState", Location.pose.toString());
        telemetry.addData("Class Hash in teleopState", Location.class.hashCode());
        telemetry.update();
        _robot.limelightHardware2Axis.servos();
        int[] initialKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PPG);
        // Towards opposite alliance loading zone
        int[] firstSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);
        // Middle spike
        int[] secondSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.PGP);
        // Towards goal
        int[] thirdSpikeKickingOrder = fireAutoKickerSeq(_TargetMotif, LimelightHardware2Axis.Motif.GPP);

        //Action trajectoryActionBuilder = _robot.driveRR.mecanumDrive.actionBuilder(_beginPos);
        int fastSpeed = 3015;

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(_beginPos)
                // starts intake and shooter
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, fastSpeed)) // speed is placeholder

                // move to shooting area


                .strafeToSplineHeading(new Vector2d(-56, 12), Math.toRadians(34))
                .waitSeconds(1.6)
                //.stopAndAdd(new AutoActions.waitForShooter(_robot, fastSpeed))
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .waitSeconds(0.1)

                //intakes first spike
                .stopAndAdd(new AutoActions.IntakeRunFast(_robot, 0.6))
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 2335))
                .strafeToSplineHeading(new Vector2d(-25, 23), Math.toRadians(47), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-5, 55), Math.toRadians(45), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.1)

                //opens gate
                .strafeToSplineHeading(new Vector2d(-3, 54), Math.toRadians(93), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(-2, 57), Math.toRadians(93), fastVelConstraint, fastAccelConstraint)

                .stopAndAdd(new AutoActions.IntakeRunFast(_robot, 0.6))

                //shoots first spike
                .waitSeconds(0.3)
                .strafeToSplineHeading(new Vector2d(-2, 20), Math.toRadians(39), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(16, 11), Math.toRadians(39), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.1)
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.6)
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .waitSeconds(0.1)

                // intakes second spike
                .strafeToSplineHeading(new Vector2d(12, 32), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
                .strafeToSplineHeading(new Vector2d(12, 58), Math.toRadians(90), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.1)

                //shoots second spike
                .strafeToSplineHeading(new Vector2d(16, 11), Math.toRadians(38 ), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.1)
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .waitSeconds(0.1)

                //intakes third spike
                .stopAndAdd(new AutoActions.SetShooterSpeed(_robot, 3005))
                .strafeToSplineHeading(new Vector2d(-46, 10), Math.toRadians(94), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.2)
                .strafeToSplineHeading(new Vector2d(-46, 62), Math.toRadians(92), fastVelConstraint, fastAccelConstraint)
                .waitSeconds(0.25)

                //shoots third spike
                .strafeToSplineHeading(new Vector2d(-56, 12), Math.toRadians(15))
                .stopAndAdd(new AutoActions.shootKickingColor(_robot, 80, _TargetMotifAction))
                .waitSeconds(0.5)
                .stopAndAdd(new AutoActions.KickerUnkick(_robot))
                .waitSeconds(0.1)

                .strafeToSplineHeading(new Vector2d(-56, 48), Math.toRadians(0));

              /*  .strafeToSplineHeading(new Vector2d(-33, 32), Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToSplineHeading(new Vector2d(-33, 62), Math.toRadians(90));
                */
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
