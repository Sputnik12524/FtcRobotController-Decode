package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500, 500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-49, -49, Math.toRadians(45)))
                        .waitSeconds(5)
                        .lineToLinearHeading(new Pose2d(-29, -25, Math.toRadians(170)))
                        .turn(Math.toRadians(45))
                        .lineToLinearHeading(new Pose2d(-7, -27, Math.toRadians(90)))

                        /*.waitSeconds(5)
                        .lineToLinearHeading(new Pose2d(-11, 27, Math.toRadians(90)))
                        .waitSeconds(5)
                        .forward(20)
                        .lineToLinearHeading(new Pose2d(-29,25, Math.toRadians(-45)))
                        .waitSeconds(5)
                        .lineToLinearHeading(new Pose2d(-50,25, Math.toRadians(0)))
                        .waitSeconds(10)*/
                        .build());

        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/Admin/Downloads/decode-custom-field-images-meepmeep-compatible-printer-v0-xsjhmvxpoonf1.png"));
        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}