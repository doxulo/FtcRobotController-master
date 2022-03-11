package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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
                                .splineToSplineHeading(new Pose2d(-12, -46, Math.toRadians(270)), Math.toRadians(90))
                                .waitSeconds(0.1)
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -68, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(43, -68, Math.toRadians(0)),0)
                                .waitSeconds(0.2)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(16,-68, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-12, -46, Math.toRadians(270)), Math.toRadians(90))
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -69, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(44, -69, Math.toRadians(0)),0)
                                .waitSeconds(0.2)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(16,-70, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-12, -44, Math.toRadians(270)), Math.toRadians(90))
                                .waitSeconds(0.1)
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -70, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(46, -70, Math.toRadians(0)),0)
                                .waitSeconds(0.2)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(16,-72, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-12, -44, Math.toRadians(270)), Math.toRadians(90))
                                .waitSeconds(0.1)
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(16, -70, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(43, -70, Math.toRadians(0)),0)
                                /**
                                 * Parked
                                 */
                                .build()
                );

        RoadRunnerBotEntity blue_cycle_no_slide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(50, 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 65, Math.toRadians(90)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-12, 42, Math.toRadians(90)), Math.toRadians(270))
                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d())
                                .build()
                );

        RoadRunnerBotEntity blue_duckbox = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50 , 50, 2 * Math.PI, 2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 65, Math.toRadians(90)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-62, 55, Math.toRadians(180)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-62, 62,Math.toRadians(200)))
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(-62,55, Math.toRadians(180)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(90)), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-31, 28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .waitSeconds(0.5)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-46, 25, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(2)
                                .splineToSplineHeading(new Pose2d(-60, 40, Math.toRadians(0)),Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity red_duckbox = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50 , 50, 2 * Math.PI,2 * Math.PI, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-62, -55, Math.toRadians(0)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-62,-62, Math.toRadians(315)))
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(-62,-55, Math.toRadians(0)))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, -30, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-27, -28, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(-46, -20, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-58, -35, Math.toRadians(0)),Math.toRadians(270))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
//                .addEntity(blue_cycle_no_slide)
                .addEntity(red_cycle)
                .addEntity(red_duckbox)
                .addEntity(blue_duckbox)
                .start();
    }
}