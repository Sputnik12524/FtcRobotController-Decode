package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Limelight;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Transfer;
import org.firstinspires.ftc.teamcode.modules.Turret;

import java.io.IOException;

public class AutoFSM {
    public enum AUTO {DEFAULT, PARK, HUMAN, GOAL}

    AUTO auto = AUTO.DEFAULT;

    enum AutoStates {DEFAULT, MOVE, SHOOT, INIT}

    public AutoStates autoState = AutoStates.DEFAULT;

    public enum MODE {AUTO, DRIVER}

    public MODE mode = MODE.DRIVER;

    Shooter sh;
    Logger logger;
    Intake in;
    Turret tt;
    Transfer tr;
    Follower follower;
    AutoSniper as;
    Limelight ll;
    Paths paths;
    ElapsedTime autoStTimer;
    ElapsedTime autoTimer;

    Alliance alliance;
    public boolean complete = false;
    public boolean wroteLogger = true;
    public boolean ready = false;


    public AutoFSM(Follower follower, Transfer tr, Shooter sh, Limelight ll, Intake in, Turret tt, Logger logger, AutoSniper as, Paths paths) {
        this.sh = sh;
        this.logger = logger;
        this.in = in;
        this.tt = tt;
        this.tr = tr;
        this.follower = follower;
        this.as = as;
        this.ll = ll;
        this.paths = paths;
        autoTimer = new ElapsedTime();
        autoStTimer = new ElapsedTime();
        readLogger();
    }

    public void readLogger() {
        try {
            logger.getAll("pospos");
            follower.setStartingPose(new Pose(logger.x, logger.y, logger.heading));
            if (logger.al == Alliance.BLUE) {
                alliance = Alliance.BLUE;
                as.setAlliance(Alliance.BLUE);
            } else {
                as.setAlliance(Alliance.RED);
                alliance = Alliance.RED;
            }
            tt.ZeroRealPose = logger.turretPose;
        } catch (IOException | NullPointerException e) {
            tt.isResetTurretPose = true;
            wroteLogger = false;
            follower.setStartingPose(new Pose(72, 72, 0));
            as.setAlliance(Alliance.BLUE); //аккуратно
        }
    }

    public void update() {
        switch (auto) {
            case GOAL:
                updateGoal();
                break;
            case HUMAN:
                updateHuman();
                break;
            case PARK:
                updatePark();
                break;
        }
    }

    void updateGoal() {
        switch (autoState) {
            case DEFAULT:
                break;
            case INIT:
                sh.shootStop();
                in.rotateStop();
                if (alliance == Alliance.BLUE)
                    follower.followPath(paths.blueGoal(follower.getPose()));
                else if (alliance == Alliance.RED)
                    follower.followPath(paths.redGoal(follower.getPose()));
                setAutoState(AutoStates.MOVE);
                break;
            case MOVE:
                if (!follower.isBusy()) setAutoState(AutoStates.SHOOT);
                if (autoStTimer.milliseconds() > 5000) {
                    follower.breakFollowing();
                    setAutoState(AutoStates.SHOOT);
                }
                break;
            case SHOOT:
                in.rotateIn();
                sh.openTunnel();
                if (autoStTimer.milliseconds() > 400) {
                    setAuto(AUTO.DEFAULT);
                    setAutoState(AutoStates.DEFAULT);
                    complete = true;
                }
                break;
        }
    }

    void updateHuman() {
        switch (autoState) {
            case DEFAULT:
                break;
            case INIT:
                sh.shootStop();
                in.rotateStop();
                if (alliance == Alliance.BLUE)
                    follower.followPath(paths.blueHuman(follower.getPose()));
                else if (alliance == Alliance.RED)
                    follower.followPath(paths.redHuman(follower.getPose()));
                setAutoState(AutoStates.MOVE);
                break;
            case MOVE:
                if (!follower.isBusy()) setAutoState(AutoStates.SHOOT);
                if (autoStTimer.milliseconds() > 5000) {
                    follower.breakFollowing();
                    setAutoState(AutoStates.SHOOT);
                }
                break;
            case SHOOT:
                in.rotateIn();
                sh.openTunnel();
                if (autoStTimer.milliseconds() > 400) {
                    setAuto(AUTO.DEFAULT);
                    setAutoState(AutoStates.DEFAULT);
                    complete = true;
                }
                break;
        }
    }

    void updatePark() {
        switch (autoState) {
            case DEFAULT:
                break;
            case INIT:
                sh.shootStop();
                in.rotateStop();
                ready = false;
                setAutoState(AutoStates.MOVE);
                break;

            case MOVE:
                if (!ready) {
                    if (alliance == Alliance.BLUE)
                        follower.followPath(paths.blueParking(follower.getPose()));
                    else
                        follower.followPath(paths.redParking(follower.getPose()));

                    ready = true;
                }

                if (follower.isBusy()) return;

                else {
                    follower.breakFollowing();
                    setAuto(AUTO.DEFAULT);
                    setAutoState(AutoStates.DEFAULT);
                    complete = true;
                }
                break;
        }
    }

    public void setAutoState(AutoStates state) {
        mode = MODE.AUTO;
        autoState = state;
        autoStTimer.reset();
    }

    public void setAuto(AUTO auto) {
        mode = MODE.AUTO;
        this.auto = auto;
        autoState = AutoStates.INIT;
        autoTimer.reset();
    }

    public void artefactsControl(){
        if (tr.howMany() > 3) {
            in.rotateStop();
        }
    }

    public void updateArtefacts() {
        if (tr.howMany() > 3) {
            in.rotateStop();
            if (follower.getPose().getY() >= 48) setAuto(AUTO.GOAL);
            else setAuto(AUTO.HUMAN);
        }
    }

}
