package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDCommands {
    int OFFSET_THRESHOLD = 20;

    double WHEEL_DIAMETER = 2;
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    double FULL_REVOLUTION_TICKS = 360;
    double LENGTH_TO_TICKS_RATIO = WHEEL_CIRCUMFERENCE/FULL_REVOLUTION_TICKS;
    ModernRoboticsI2cGyro gyro;

    DcMotorEx RF;
    DcMotorEx LF;
    DcMotorEx RB;
    DcMotorEx LB;

    // TODO: find max velocity for wheels
    double RF_MAX_VELOCITY;
    double LF_MAX_VELOCITY;
    double RB_MAX_VELOCITY;
    double LB_MAX_VELOCITY;

    double bias = 0.80D;

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

    public PIDCommands(
            ModernRoboticsI2cGyro gyro, 
            DcMotorEx RF, 
            DcMotorEx LF, 
            DcMotorEx RB, 
            DcMotorEx LB, 
            PIDController movementController,
            PIDController turnController
    ) {
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

    private double inchesToTicks(double inches) {
        return inches/this.LENGTH_TO_TICKS_RATIO;
    }

    private boolean isBusy(int motorPosition, int target) {
        return Math.abs(motorPosition-target) < this.OFFSET_THRESHOLD;
    }

    private void setTurningVelocity(DcMotorEx motor, int currentAngle, int targetAngle, int multiple) {
        if (turnController.paused) {
            turnController.resume();
        }
        motor.setVelocity(
                Math.min(
                    turnController.calculate(targetAngle, currentAngle)*multiple,
                    this.MAX_VELOCITY
                )
        );
    }
    private void setVelocity(DcMotorEx motor, int currentPosition, int targetPosition) {
        if (movementController.paused) {
            movementController.resume();
        }
        motor.setVelocity(
                Math.min(
                        movementController.calculate(targetPosition, currentPosition),
                        this.MAX_VELOCITY
                )
        );
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
            this.setVelocity(this.RF, RFCurrentPosition, RFTarget);
            this.setVelocity(this.LF, LFCurrentPosition, LFTarget);
            this.setVelocity(this.RB, RBCurrentPosition, RBTarget);
            this.setVelocity(this.LB, LBCurrentPosition, LBTarget);
        }
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
            this.setVelocity(this.RF, RFCurrentPosition, RFTarget);
            this.setVelocity(this.LF, LFCurrentPosition, LFTarget);
            this.setVelocity(this.RB, RBCurrentPosition, RBTarget);
            this.setVelocity(this.LB, LBCurrentPosition, LBTarget);
        }
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

        turnController.pauseAndReset();
        return this;
    }

    public PIDCommands turnLeft(int theta) {
        return this.turnRight(-theta);
    }

    public PIDCommands gotoEncoderPosition(DcMotorEx[] motors, int[] encoderPositions) {

        return this;
    }
}
