package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 10.80737288830478)
                .build();

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-72, -24, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());*/

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -24, Math.PI))

                // move to shooting area

                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)


                .waitSeconds(2)

                // moves to first spike to intake artifacts, then moves back
                .splineTo(new Vector2d(-12, -48), -(Math.PI) / 4)
                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                .waitSeconds(2)

                // moves to second spike to intake artifacts, then moves back
                .splineTo(new Vector2d(12, -48), -(Math.PI)/4)
                .splineTo(new Vector2d(-12, -12), -(3*Math.PI)/4)

                // kicks artifacts from second spike

                .build());*/

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(67, -15.5, Math.PI))
                // move to shooting area

                .strafeToSplineHeading(new Vector2d(-12, -12), -(3*Math.PI)/4)


                .waitSeconds(2)


                //.waitSeconds(2)

                // moves to second spike to intake artifacts, then moves back
                .strafeToSplineHeading(new Vector2d(12, -24), -(Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(36, -48), -(Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(12, -24), -(3*Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-12, -12), -(3*Math.PI)/4)

                .waitSeconds(2)

                .strafeToSplineHeading(new Vector2d(12, -48), -(Math.PI)/4)
                .strafeToSplineHeading(new Vector2d(-12, -12), -(3*Math.PI)/4)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}