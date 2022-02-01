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
                        drive.trajectorySequenceBuilder(new Pose2d(13, 65, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-11, 43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13, 66, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(50, 66))
                                .lineToConstantHeading(new Vector2d(13, 66))
                                .lineToLinearHeading(new Pose2d(-11, 43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13, 66, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(50, 66))
                                .lineToConstantHeading(new Vector2d(13, 66))
                                .lineToLinearHeading(new Pose2d(-11, 43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13, 66, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(50, 66))
                                .lineToConstantHeading(new Vector2d(13, 66))
                                .lineToLinearHeading(new Pose2d(-11, 43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13, 66, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(50, 66))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}