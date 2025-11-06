package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTestRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500, 500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 7, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(36, 30, Math.toRadians(90)))
                        //нужно написать взятие артефактов
                        //           .lineToLinearHeading(new Pose2d(-29, 30, Math.toRadians(135)))
                        //          .waitSeconds(2)
                        .build());

        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\Sputnik MSI laptop 1\\Downloads\\324f5ccd-53dc-4872-ba37-648e96fb7d47.tmp"));
        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
