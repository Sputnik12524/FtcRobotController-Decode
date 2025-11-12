package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTestRedGoal {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500, 500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-48, 48, Math.toRadians(-45)))
                        .lineToLinearHeading(new Pose2d(0, 0, 110))
                        .waitSeconds(3)
                        .lineToLinearHeading(new Pose2d(-56, -2,90))
                        .build());


        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/Sputnik MSI laptop 1/Downloads/decode-custom-field-images-meepmeep-compatible-printer-v0-xsjhmvxpoonf1.png"));
        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
