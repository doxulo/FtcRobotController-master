package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;

import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.font.TrueTypeFont;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 10, 3.14, Math.toRadians(90), 30)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(25, 18), Math.toRadians(90))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(5, -1.5, Math.toRadians(180)))
                                .setReversed(true)
                                .forward(33)
                                .strafeTo(new Vector2d(-33, 10))
                                .strafeTo(new Vector2d(-28, -1.5))
                                .back(35)
                                .splineToSplineHeading(new Pose2d(25, 18, Math.toRadians(270)), Math.toRadians(90))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(5, -1.5, Math.toRadians(180)), Math.toRadians(200))
                                .setReversed(true)
                                .forward(30)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}