package org.firstinspires.ftc.teamcode.util;

public class PIDController {

    double[] biasPoints;

    public double summation = 0;
    double fullRotation;
    double offset;

    public double kP;
    public double kI;
    public double kD;

    long lastTime;

    boolean paused;

    public PIDController(double kP, double kI, double kD, double[] biasPoints, double fullRotation, double offset) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.biasPoints = biasPoints;
        this.fullRotation = fullRotation;
        this.offset = offset;
        this.paused = true;
    }

    public double lerp(double p0, double p1, double t) {
        return (1 - t)*p0 + p1*t;
    }

    public double calculate(double sp, double pv) {
        if (paused) {
            this.resume();
        }

        double dt = (double) (System.currentTimeMillis() - lastTime);
        double bias = lerp(this.biasPoints[0], this.biasPoints[1], pv/this.fullRotation);
        double dp = sp-pv;
        double derivative = dt == 0 ? 0 : dp/dt;

        this.summation = summation + dp*dt;
        lastTime = System.currentTimeMillis();

        return dp*this.kP + this.summation*this.kI + derivative*this.kD + bias;
    }

    public void pauseAndReset() {
        this.summation = 0;
        this.pause();
    }

    public void pause() {
        this.paused = true;
    }

    public void resume() {
        this.paused = false;
        this.lastTime = System.currentTimeMillis();
    }
    /*
    1. Kp = 0.01, Ki = 0, Kd = 0. Tune Kp until it is a bit too much (oscillates a bit).
    2. Increase Kd until it stabilizes the controller
    3. Increase integral to fix steady state

    -If really jittery around setpoint: Kd too high
    -Wide oscillations: Kp too high
     */
}
