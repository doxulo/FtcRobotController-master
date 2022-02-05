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
                .setConstraints(40, 54, 4.02694535445, 4.02694535445, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-11, -43, Math.toRadians(270)), Math.toRadians(160))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(50, -65, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(true)
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-11, -43, Math.toRadians(270)), Math.toRadians(90))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(50, -65, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(true)
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-11, -43, Math.toRadians(270)), Math.toRadians(90))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(50, -65, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(true)
                                .waitSeconds(0.1)
                                .splineToSplineHeading(new Pose2d(-11, -43, Math.toRadians(270)), Math.toRadians(90))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(50, -65, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}