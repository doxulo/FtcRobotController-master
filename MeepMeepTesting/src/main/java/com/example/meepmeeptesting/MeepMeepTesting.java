package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;

import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.font.TrueTypeFont;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity blue_cycle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(50 , 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 65, Math.toRadians(90)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                                .build()
                );
        RoadRunnerBotEntity red_cycle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(50 , 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10, -55, Math.toRadians(310)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,-65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, -65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,-65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, -55, Math.toRadians(310)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,-65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, -65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,-65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, -55, Math.toRadians(310)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,-65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, -65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,-65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, -55, Math.toRadians(310)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,-65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, -65, Math.toRadians(0)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10,-65 , Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(10, -55, Math.toRadians(310)))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(10,-65, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(40, -65, Math.toRadians(0)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(blue_cycle)
                .addEntity(red_cycle)
                .start();
    }
}