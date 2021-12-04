package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    double[] biasPoints;

    public double summation = 0;
    double fullRotation;
    double offset;

    public double kP;
    public double kI;
    public double kD;

    long lastTime;

    public boolean paused;

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

    public double calculate(double sp, double pv, Telemetry t) {
        if (paused) {
            this.resume();
        }

        double dt = (double) (System.currentTimeMillis() - lastTime);
        double bias = lerp(this.biasPoints[0], this.biasPoints[1], (pv+offset)/this.fullRotation);
        double dp = sp-pv;
        double gradient = dt == 0 ? 0 : dp/dt;

        double error = dp*this.kP;
        double integral = this.summation*this.kI;
        double derivative = gradient*this.kD;

        this.summation = summation + dp*dt;
        lastTime = System.currentTimeMillis();

        if (integral > 0.05) {
            integral = 0.05;
        } else if (integral < -0.15) {
            integral = -0.15;
        }

        t.addLine(String.format("%f, %f, %f, %f", error, integral, derivative, error+integral+derivative));
        return error + integral + derivative + bias;
    }

    public double calculate(double sp, double pv) {
        if (paused) {
            this.resume();
        }

        double dt = (double) (System.currentTimeMillis() - lastTime);
        double bias = this.fullRotation == 0 ? 0 : lerp(this.biasPoints[0], this.biasPoints[1], (pv+offset)/this.fullRotation);
        double dp = sp-pv;
        double gradient = dt == 0 ? 0 : dp/dt;

        double error = dp*this.kP;
        double integral = this.summation*this.kI;
        double derivative = gradient*this.kD;

        this.summation = summation + dp*dt;
        lastTime = System.currentTimeMillis();

        return (double) (error + integral + derivative + bias);
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

    public double getFloat(double theta) {
        return lerp(this.biasPoints[0], this.biasPoints[1], (theta+offset)/this.fullRotation);
    }
    /*
    1. Kp = 0.01, Ki = 0, Kd = 0. Tune Kp until it is a bit too much (oscillates a bit).
    2. Increase Kd until it stabilizes the controller
    3. Increase integral to fix steady state

    -If really jittery around setpoint: Kd too high
    -Wide oscillations: Kp too high
     */
}
