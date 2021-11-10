package org.firstinspires.ftc.teamcode.util;

public class DebounceObject {

    private final String name;
    private long lastTime = 0;
    private double allottedTime;

    public DebounceObject(String name, double allottedTime) {
        this.name = name;
        this.allottedTime = allottedTime;
    }

    public String getName() {
        return this.name;
    }

    public long getLastTime() {
        return this.lastTime;
    }

    public boolean check() {
        return System.currentTimeMillis() - this.lastTime > this.allottedTime;
    }

    public void reset() {
        this.lastTime = System.currentTimeMillis();
    }

    public boolean updateLastTime() {
        this.lastTime = System.currentTimeMillis();
        return true;
    }
}
