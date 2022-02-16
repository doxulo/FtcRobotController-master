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
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-29, 10, Math.toRadians(0)))
                                .strafeRight(6)
                                .waitSeconds(3)
                                .strafeLeft(8)
                                .turn(Math.toRadians(-90))
                                .back(35)
                                .turn(Math.toRadians(-100))
                                .lineToLinearHeading(new Pose2d(-2,39, Math.toRadians(180)))
            .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-20, 42))
                                .lineToLinearHeading(new Pose2d(-30, 25.5, Math.toRadians(0)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}