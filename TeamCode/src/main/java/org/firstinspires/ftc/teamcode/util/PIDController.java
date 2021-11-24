package org.firstinspires.ftc.teamcode.util;

public class PIDController {

    double[] biasPoints;

    double summation = 0;
    double fullRotation;
    double offset;

    double kp;
    double ki;
    double kd;

    long lastTime = System.currentTimeMillis();

    public PIDController(double kp, double ki, double kd, double[] biasPoints, double fullRotation, double offset) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.biasPoints = biasPoints;
        this.fullRotation = fullRotation;
        this.offset = 0;
    }

    public double lerp(double p0, double p1, double t) {
        return (1-t)*p0 + p1*t;
    }

    public double calculate(double sp, double pv) {
        int dt = (int) (System.currentTimeMillis() - lastTime);
        double dp = sp-pv;
        this.summation = summation + dp*dt;
        double derivative = dp/dt;

        double bias = lerp(this.biasPoints[0], this.biasPoints[1], (pv+this.offset)/this.fullRotation);
        lastTime = System.currentTimeMillis();
        return dp*this.kp + this.summation*this.ki + derivative*this.kd + bias;
    }

    /*
    1. Kp = 0.01, Ki = 0, Kd = 0. Tune Kp until it is a bit too much (oscillates a bit).
    2. Increase Kd until it stabilizes the controller
    3. Increase integral to fix steady state

    -If really jittery around setpoint: Kd too high
    -Wide oscillations: Kp too high
     */
}
