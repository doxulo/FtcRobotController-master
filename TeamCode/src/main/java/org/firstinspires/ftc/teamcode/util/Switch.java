package org.firstinspires.ftc.teamcode.util;

public class Switch {

    private boolean defaultState = false;
    private boolean currentState = false;

    public Switch(boolean defaultState) {
        this.defaultState = defaultState;
        this.currentState = defaultState;
    }

    public Switch(boolean defaultState, boolean currentState) {
        this.defaultState = defaultState;
        this.currentState = currentState;
    }

    public boolean check() {
        return this.currentState;
    }

    public void reset() {
        this.currentState = this.defaultState;
    }

    public void setTrue() {
        this.set(true);
    }

    public void setFalse() {
        this.set(false);
    }

    public void set(boolean state) {
        this.currentState = state;
    }

    public void trigger() {
        this.defaultState = !this.currentState;
    }

    public void checkAndTrigger() {
        if (this.check()) {
            this.trigger();
        }
    }
}
