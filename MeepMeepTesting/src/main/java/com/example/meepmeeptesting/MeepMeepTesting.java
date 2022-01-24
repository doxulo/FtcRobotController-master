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
                                .lineToLinearHeading(new Pose2d(19, 10, Math.toRadians(180)))
                                .strafeLeft(6)
                                .waitSeconds(3)
                                .forward(5)
                                .turn(Math.toRadians(80))
                                .back(15)
                                .splineToLinearHeading(new Pose2d(-10,36, Math.toRadians(10)), Math.toRadians(190))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20, 26.5, Math.toRadians(270)), Math.toRadians(315))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}