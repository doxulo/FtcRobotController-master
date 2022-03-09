package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
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
                .setConstraints(50, 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 65, Math.toRadians(90)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(50)))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, 63.5, Math.toRadians(5)), Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, 67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, 55, Math.toRadians(50)), Math.toRadians(230))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, 63.5, Math.toRadians(5)), Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, 67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, 55, Math.toRadians(50)), Math.toRadians(230))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, 63.5, Math.toRadians(5)), Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, 67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, 55, Math.toRadians(50)), Math.toRadians(230))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, 63.5, Math.toRadians(5)), Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, 67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, 55, Math.toRadians(50)), Math.toRadians(230))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, 63.5, Math.toRadians(5)), Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(22,67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, 67, Math.toRadians(0)),0)
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
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -63.5, Math.toRadians(355)), Math.toRadians(310))
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, -67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, -55, Math.toRadians(310)), Math.toRadians(130))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -63.5, Math.toRadians(355)), Math.toRadians(310))
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, -67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, -55, Math.toRadians(310)), Math.toRadians(130))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -63.5, Math.toRadians(355)), Math.toRadians(310))
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, -67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, -55, Math.toRadians(310)), Math.toRadians(130))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -63.5, Math.toRadians(355)), Math.toRadians(310))
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, -67, Math.toRadians(0)),0)
                                .waitSeconds(.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, -55, Math.toRadians(310)), Math.toRadians(130))
                                .waitSeconds(.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -63.5, Math.toRadians(355)), Math.toRadians(310))
                                .splineToSplineHeading(new Pose2d(22,-67, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(40, -67, Math.toRadians(0)),0)
                                .build()
                );
        RoadRunnerBotEntity blue_duckbox = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50 , 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 65, Math.toRadians(270)))
                                .splineToSplineHeading(new Pose2d(-62, 62, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(2)
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-55, 25, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(-62, 40, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity red_duckbox = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50 , 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-65, -65, Math.toRadians(0)), Math.toRadians(180))
                                .waitSeconds(2)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-60, -45, Math.toRadians(180)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-65, -25, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-65, -35))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(blue_cycle)
                .addEntity(red_cycle)
                .addEntity(red_duckbox)
                .addEntity(blue_duckbox)
                .start();
    }
}