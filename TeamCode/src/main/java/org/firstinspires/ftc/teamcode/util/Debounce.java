package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;

public class Debounce {

    private final HashMap<String, DebounceObject> debounces = new HashMap<>();

    public Debounce(double debounceTime, String... names) {
        for (int i = 0; i < names.length; i++) {
            this.add(names[i], debounceTime);
        }
    }

    public Debounce(DebounceObject... objects) {
        for (int i = 0; i < objects.length; i++) {
            this.add(objects[i]);
        }
    }

    private DebounceObject getObjectAssert(String name) {
        DebounceObject object = this.debounces.get(name);
        assert object != null;
        return object;
    }

    public void add(DebounceObject object) {
        this.debounces.put(
                object.getName(),
                object
        );
    }

    public void add(String name, double debounceTime) {
        this.add(
                new DebounceObject(name, debounceTime)
        );
    }

    public long getTimePassed(String name) {
        return System.currentTimeMillis() - this.getObjectAssert(name).getLastTime();
    }

    public boolean getTimePassedAndCheck(String name, long comparisonTime) {
        return this.getTimePassed(name) >= comparisonTime;
    }

    public boolean checkAndUpdate(String name) {
        return this.check(name) && this.update(name);
    }

    public boolean check(String name) {
        return this.getObjectAssert(name).check();
    }

    public boolean update(String name) {
        return this.getObjectAssert(name).updateLastTime();
    }

    public void reset(String name) {
        this.getObjectAssert(name).reset();
    }

    public boolean checkIfExist(String name) {
        return !(this.debounces.get(name) == null);
    }
}
