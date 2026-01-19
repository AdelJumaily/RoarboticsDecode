package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {

//        final Vector2d path0 = new Vector2d(-24,48);
//        final Vector2d path1 = new Vector2d(-36, 48);
//        final Vector2d path2 = new Vector2d(-12,-12);
//        final Vector2d path3 = new Vector2d(-60,12);
//        final Vector2d path4 = new Vector2d(-36,-12);
//        this.drive = new DCBaseLIS(hardwareMap);
//        drive.setPoseEstimate(startPoseRL);
//        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
//                .lineTo(path0) // MTSP
//                .build();
//
//
//        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
//                .lineTo(path1)
//                .build();
//
//
//        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
//                .lineTo(path2)
//                .build();
//
//        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
//                .lineTo(path3)
//                .build();
//
//        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
//                .lineTo(path4)
//                .turn(35)
//                .build();

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50.5, 50.25, Math.toRadians(120)))
//                .waitSeconds(2)
//                .lineToY(24)
//                .waitSeconds(2)
//                .turn(Math.toRadians(-120))
//                .lineToX(-12)
//                .waitSeconds(2)
//                .turn(Math.toRadians(90))
//                .lineToY(32)
//                .waitSeconds(2)
//                .lineToY(50)
//                .turn(Math.toRadians(137))
//                .lineToY(24)
//                .turn(Math.toRadians(-100))
                .lineToX(-24)
                .turn(Math.toRadians(-30))
                .lineToY(24)
                .turn(Math.toRadians(225))
                .turn(Math.toRadians(45))
                //.turn(Math.toRadians(-135))
                .lineToX(-12)
                .turn(Math.toRadians(90))
                .lineToY(32)
                .lineToY(50)
                .turn(Math.toRadians(155))
                .lineToY(24)
                .turn(Math.toRadians(-115+180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}