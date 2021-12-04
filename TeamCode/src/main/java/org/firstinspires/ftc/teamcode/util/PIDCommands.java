package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class PIDCommands {
    int OFFSET_THRESHOLD = 1;

    public static double WHEEL_DIAMETER = 2;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    public static double FULL_REVOLUTION_TICKS = 537;
    public static double LENGTH_TO_TICKS_RATIO = WHEEL_CIRCUMFERENCE/FULL_REVOLUTION_TICKS;
    ModernRoboticsI2cGyro gyro;

    DcMotorEx RF;
    DcMotorEx LF;
    DcMotorEx RB;
    DcMotorEx LB;

    // TODO: find max velocity for wheels
    double RF_MAX_VELOCITY = 2000;
    double LF_MAX_VELOCITY = 2000;
    double RB_MAX_VELOCITY = 2000;
    double LB_MAX_VELOCITY = 2000;

    double bias = 0.50D;

    double MAX_VELOCITY = Math.min(
            Math.min(RF_MAX_VELOCITY, LF_MAX_VELOCITY),
            Math.min(RB_MAX_VELOCITY, LB_MAX_VELOCITY)
    )*bias;

    DcMotorEx[] allMotors;
    DcMotorEx[] leftMotors;
    DcMotorEx[] rightMotors;

    DcMotorEx[] forwardMotors;
    DcMotorEx[] backwardMotors;

    PIDController movementController;
    PIDController turnController;
    
    boolean adjustingOrientation = true;

    Telemetry t;

    public PIDCommands(
            ModernRoboticsI2cGyro gyro, 
            DcMotorEx RF, 
            DcMotorEx LF, 
            DcMotorEx RB, 
            DcMotorEx LB, 
            PIDController movementController,
            PIDController turnController,
            Telemetry t
    ) {
        this.t = t;
        this.gyro = gyro;
        this.RF = RF;
        this.LF = LF;
        this.RB = RB;
        this.LB = LB;
        this.allMotors = new DcMotorEx[] {
                RF, RB, LF, LB
        };
        this.rightMotors = new DcMotorEx[] {
                RF, RB
        };
        this.leftMotors = new DcMotorEx[] {
                LF, LB
        };
        this.forwardMotors = new DcMotorEx[] {
                RF
        };
        this.backwardMotors = new DcMotorEx[] {
                LF, LB, RB
        };
        this.movementController = movementController;
        this.turnController = turnController;
        this.movementController.pauseAndReset();
        this.turnController.pauseAndReset();
    }

    public static double inchesToTicks(double inches) {
        return inches/LENGTH_TO_TICKS_RATIO;
    }

    private boolean isBusy(int motorPosition, int target) {
        return Math.abs(motorPosition-target) < this.OFFSET_THRESHOLD;
    }

    private void setTurningVelocity(DcMotorEx motor, int currentAngle, int targetAngle, int multiple) {
        if (turnController.paused) {
            turnController.resume();
        }

        double calculatedVelocity = turnController.calculate(targetAngle, currentAngle, this.t)*multiple;
        if (calculatedVelocity < 0) {
            calculatedVelocity = Math.max(calculatedVelocity, -this.MAX_VELOCITY);
        } else {
            calculatedVelocity = Math.min(calculatedVelocity, this.MAX_VELOCITY);
        }
        this.t.addData("Power: ", calculatedVelocity);
        motor.setVelocity(
                calculatedVelocity
        );
    }
    private void setVelocity(DcMotorEx motor, int currentPosition, int targetPosition) {
        if (movementController.paused) {
            movementController.resume();
        }

        double calculatedVelocity = movementController.calculate((double)targetPosition, (double) currentPosition, this.t);
        this.t.addLine(String.format("Power: %f", calculatedVelocity));
        /*if (calculatedVelocity < 0) {
            calculatedVelocity = Math.max(calculatedVelocity, -this.MAX_VELOCITY);
        } else {
            calculatedVelocity = Math.min(calculatedVelocity, this.MAX_VELOCITY);
        }*/
        this.t.addData("Power: ", calculatedVelocity);
        motor.setVelocity(
                calculatedVelocity
        );
    }

    private void resetVelocity() {
        this.RF.setVelocity(0);
        this.RB.setVelocity(0);
        this.LF.setVelocity(0);
        this.LB.setVelocity(0);
    }

    public void async() {
    }
    
    public int[] getEncoderPositions() {
        int[] encoderPositions = new int[this.allMotors.length];

        for (int i = 0; i < encoderPositions.length; i++) {
            encoderPositions[i] = this.allMotors[i].getCurrentPosition();
        }

        return encoderPositions;
    }

    public PIDCommands forward(double inches) {
        int RFCurrentPosition = this.RF.getCurrentPosition();
        int LFCurrentPosition = this.LF.getCurrentPosition();
        int RBCurrentPosition = this.RB.getCurrentPosition();
        int LBCurrentPosition = this.LB.getCurrentPosition();

        int RFTarget = (int) (RFCurrentPosition + this.inchesToTicks(inches));
        int LFTarget = (int) (LFCurrentPosition - this.inchesToTicks(inches));
        int RBTarget = (int) (RBCurrentPosition - this.inchesToTicks(inches));
        int LBTarget = (int) (LBCurrentPosition - this.inchesToTicks(inches));

        movementController.resume();
        while (
                !this.isBusy(RFCurrentPosition, RFTarget) &&
                !this.isBusy(LFCurrentPosition, LFTarget) &&
                !this.isBusy(RBCurrentPosition, RBTarget) &&
                !this.isBusy(LBCurrentPosition, LBTarget)
        ) {
            this.t.addLine(String.format("%s, %d, %d", this.RF.getDeviceName(), this.RF.getCurrentPosition(), RFTarget));
            this.t.addLine(String.format("%s, %d, %d", this.LF.getDeviceName(), this.LF.getCurrentPosition(), RFTarget));
            this.t.addLine(String.format("%s, %d, %d", this.LB.getDeviceName(), this.LB.getCurrentPosition(), RFTarget));
            this.t.addLine(String.format("%s, %d, %d", this.RB.getDeviceName(), this.RB.getCurrentPosition(), RFTarget));

            this.t.addLine(String.format("%s, Velocity: %f, %f", this.RF.getDeviceName(), this.RF.getVelocity(), this.RF.getPower()));
            this.t.addLine(String.format("%s, Velocity: %f, %f", this.LF.getDeviceName(), this.LF.getVelocity(), this.LF.getPower()));
            this.t.addLine(String.format("%s, Velocity: %f, %f", this.RB.getDeviceName(), this.RB.getVelocity(), this.RB.getPower()));
            this.t.addLine(String.format("%s, Velocity: %f, %f", this.LB.getDeviceName(), this.LB.getVelocity(), this.LB.getPower()));

            this.setVelocity(this.RF, RFCurrentPosition, RFTarget);
            this.setVelocity(this.LF, LFCurrentPosition, LFTarget);
            this.setVelocity(this.RB, RBCurrentPosition, RBTarget);
            this.setVelocity(this.LB, LBCurrentPosition, LBTarget);
            RFCurrentPosition = RF.getCurrentPosition();
            LFCurrentPosition = LF.getCurrentPosition();
            RBCurrentPosition = RB.getCurrentPosition();
            LBCurrentPosition = LB.getCurrentPosition();
            
            this.t.update();

        }
        this.resetVelocity();
        movementController.pauseAndReset();
        return this;
    }

    public PIDCommands backward(double inches) {
        return this.forward(-inches);
    }

    public PIDCommands strafeLeft(double inches) {
        int RFCurrentPosition = this.RF.getCurrentPosition();
        int LFCurrentPosition = this.LF.getCurrentPosition();
        int RBCurrentPosition = this.RB.getCurrentPosition();
        int LBCurrentPosition = this.LB.getCurrentPosition();

        int RFTarget = (int) (RFCurrentPosition + this.inchesToTicks(inches));
        int LFTarget = (int) (LFCurrentPosition + this.inchesToTicks(inches));
        int RBTarget = (int) (RBCurrentPosition - this.inchesToTicks(inches));
        int LBTarget = (int) (LBCurrentPosition + this.inchesToTicks(inches));

        movementController.resume();
        while (
                !this.isBusy(RFCurrentPosition, RFTarget) &&
                !this.isBusy(LFCurrentPosition, LFTarget) &&
                !this.isBusy(RBCurrentPosition, RBTarget) &&
                !this.isBusy(LBCurrentPosition, LBTarget)
        ) {
            this.setVelocity(this.RF, RFCurrentPosition, RFTarget);
            this.setVelocity(this.LF, LFCurrentPosition, LFTarget);
            this.setVelocity(this.RB, RBCurrentPosition, RBTarget);
            this.setVelocity(this.LB, LBCurrentPosition, LBTarget);
            RFCurrentPosition = RF.getCurrentPosition();
            LFCurrentPosition = LF.getCurrentPosition();
            RBCurrentPosition = RB.getCurrentPosition();
            LBCurrentPosition = LB.getCurrentPosition();
        }
        this.resetVelocity();
        movementController.pauseAndReset();
        return this;
    }

    public PIDCommands strafeRight(double inches) {
        return this.strafeLeft(-inches);
    }

    public PIDCommands turnRight(int theta) {

        int startHeading = this.gyro.getIntegratedZValue();
        int targetHeading = startHeading + theta;
        int error = targetHeading-startHeading;

        turnController.resume();

        while (Math.abs(error) != 0) {
            this.setTurningVelocity(this.RF, error, targetHeading, -1);
            this.setTurningVelocity(this.LF, error, targetHeading, 1);
            this.setTurningVelocity(this.RB, error, targetHeading, -1);
            this.setTurningVelocity(this.LB, error, targetHeading, 1);
            // set power to error*kP (Possible PID controller usage)
        }
        this.resetVelocity();
        turnController.pauseAndReset();
        return this;
    }

    public PIDCommands turnLeft(int theta) {
        return this.turnRight(-theta);
    }

    public PIDCommands gotoEncoderPosition(HashMap<String, Integer>[] encoderPosition) {
        for (HashMap<String, Integer> position : encoderPosition) {

        }

        return this;
    }
}
