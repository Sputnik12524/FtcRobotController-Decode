package org.firstinspires.ftc.teamcode.modules;

public class ButtonsHandler {
    private boolean previousState = false;
    private boolean currentState = false;
    private long pressedTime = 0;
    private long releasedTime = 0;
    private boolean toggleState = false;

    public void update(boolean currentState) {
        this.previousState = this.currentState;
        this.currentState = currentState;

        if (isPressed()) {
            pressedTime = System.currentTimeMillis();
            toggleState = !toggleState; // Меняем состояние тумблера
        }

        if (isReleased()) {
            releasedTime = System.currentTimeMillis();
        }
    }

    public boolean isPressed() {
        return currentState && !previousState;
    }

    public boolean isReleased() {
        return !currentState && previousState;
    }


    public boolean isDown() {
        return currentState;
    }


    public boolean isHeld() {
        return currentState && previousState;
    }

    public long getHoldTime() {
        if (!currentState || pressedTime == 0) return 0;
        return System.currentTimeMillis() - pressedTime;
    }


    public boolean isHeldFor(long milliseconds) {
        if (milliseconds <= 0) return false;
        return isHeld() && getHoldTime() >= milliseconds;
    }


    public boolean getToggleState() {
        return toggleState;
    }

    public void setToggleState(boolean state) {
        this.toggleState = state;
    }

    public long getReleasedTime() {
        return releasedTime;
    }

    public void reset() {
        previousState = false;
        currentState = false;
        pressedTime = 0;
        releasedTime = 0;
        toggleState = false;
    }
}