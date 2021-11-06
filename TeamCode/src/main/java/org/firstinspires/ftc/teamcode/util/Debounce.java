package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;

public class Debounce {

    private final HashMap<String, double[]> debounces = new HashMap<>();
    public Debounce (double debounceTime, String... names) {
        for (int i = 0; i < names.length; i++) {
            this.add(names[i], debounceTime);
        }
    }
    public void add(String name, double debounceTime) {
        this.debounces.put(
                name,
                new double[]{0, debounceTime}
        );
    }

    public boolean checkAndUpdate(String name) {
        return this.check(name) && this.update(name);
    }

    public boolean check(String name) {
        double[] keyValue = this.debounces.get(name);
        assert keyValue != null;
        return System.currentTimeMillis() - keyValue[0] > keyValue[1];
    }

    public boolean update(String name) {
        double[] keyValue = this.debounces.get(name);
        assert keyValue != null;
        keyValue[0] = System.currentTimeMillis();
        this.debounces.put(
                name,
                keyValue
        );

        return true;
    }
}
