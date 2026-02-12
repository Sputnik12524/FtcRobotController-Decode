package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.Shooter;

public class AutoSniper {
    Shooter sh;
    Turret tt;
    Follower follower;

    //---------------------------------------------- GENERAL COEFFICIENTS
    public double gateY = 144;
    public double gateX;




    public AutoSniper() {
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                gateX = 0;
                break;
            case RED:
                gateX = 144;
                break;
        }
    }

}
