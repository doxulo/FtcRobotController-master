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
                .setConstraints(54, 50, 2, 2, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-10, -45))
                                .lineToLinearHeading(new Pose2d(13, -67, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(50, -65))
                                .lineToConstantHeading(new Vector2d(13, -67))
                                .lineToLinearHeading(new Pose2d(-10, -45, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(13, -67, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(50, -65))
                                .lineToConstantHeading(new Vector2d(13, -67))
                                .lineToLinearHeading(new Pose2d(-10, -45, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(13, -67, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(50, -65))
                                .lineToConstantHeading(new Vector2d(13, -67))
                                .lineToLinearHeading(new Pose2d(-10, -45, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(13, -67, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(50, -65))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}