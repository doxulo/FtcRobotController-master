package org.firstinspires.ftc.teamcode.util;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

public class Scheduler {
    ArrayList<Long> exhaustTime = new ArrayList<>();
    ArrayList<Method> callbacks = new ArrayList<>();
    ArrayList<Object> objects = new ArrayList<>();
    ArrayList<Integer> parameters = new ArrayList<>();
    long currentTime;

    public void checkAndExecute() throws InvocationTargetException, IllegalAccessException {
        if (!(this.exhaustTime.size() == 0)) {
            this.currentTime = System.currentTimeMillis();
            if (this.exhaustTime.get(0) <= this.currentTime) {
                this.callbacks.get(0).invoke(this.objects.get(0), this.parameters.get(0));
                this.callbacks.remove(0);
                this.objects.remove(0);
                this.exhaustTime.remove(0);
                this.parameters.remove(0);

                this.checkAndExecute();
            }
        }
    }

    public int getIndex(long objectTime) {
        for (int i = 0; i < exhaustTime.size(); i++) {
            if (objectTime <= exhaustTime.get(i)) {
                return i;
            }

        }
        return exhaustTime.size();
    }

    public void add(Method method, Object object, int parameter, long callbackTime) {
        if (callbackTime < System.currentTimeMillis()) {
            callbackTime = System.currentTimeMillis() + callbackTime;
        }

        int index = getIndex(callbackTime);
        this.exhaustTime.add(index, callbackTime);
        this.callbacks.add(index, method);
        this.objects.add(index, object);
        this.parameters.add(index, parameter);
    }
}
