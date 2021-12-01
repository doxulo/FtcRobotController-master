package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

public class Commands {
    double WHEEL_DIAMETER = 2;
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    double FULL_REVOLUTION_TICKS = 360;
    // LENGTH_TO_TICKS_RATIO * inches = wheel ticks to reach that position
    double LENGTH_TO_TICKS_RATIO = WHEEL_CIRCUMFERENCE/FULL_REVOLUTION_TICKS;

    ModernRoboticsI2cGyro gyro;

    DcMotorEx RF;
    DcMotorEx LF;
    DcMotorEx RB;
    DcMotorEx LB;

    DcMotorEx[] allMotors;
    DcMotorEx[] leftMotors;
    DcMotorEx[] rightMotors;

    ArrayList<DcMotorEx> forwardMotors;
    ArrayList<DcMotorEx> backwardMotors;

    boolean adjustingOrientation = true;

    public Commands(ModernRoboticsI2cGyro gyro, DcMotorEx RF, DcMotorEx LF, DcMotorEx RB, DcMotorEx LB) {
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

        for (int i = 0; i < allMotors.length; i++) {
            switch (allMotors[i].getDirection()) {
                case FORWARD:
                    forwardMotors.add(allMotors[i]);
                    break;
                case REVERSE:
                    backwardMotors.add(allMotors[i]);
                    break;
            }
        }

    }

    private double inchesToTicks(double inches) {
        return inches/this.LENGTH_TO_TICKS_RATIO;
    }

    private void setupAllMotors(int encoderTicks) {
        for (DcMotorEx motor : this.forwardMotors) {
            motor.setTargetPosition(motor.getCurrentPosition() + encoderTicks);
        }

        for (DcMotorEx motor : this.backwardMotors) {
            motor.setTargetPosition(motor.getCurrentPosition() + (-encoderTicks));
        }
    }

    private void setupLeftRightDiagonal(int encoderTicks) {
        this.LF.setTargetPosition(this.LF.getCurrentPosition() + encoderTicks);
        this.RB.setTargetPosition(this.RB.getCurrentPosition() + encoderTicks);
    }


    private void setupRightLeftDiagonal(int encoderTicks) {
        this.LB.setTargetPosition(this.LB.getCurrentPosition() + encoderTicks);
        this.RF.setTargetPosition(this.RF.getCurrentPosition() + encoderTicks);
    }

    public void async() {
        for (DcMotorEx motor : this.allMotors) {
            while (motor.isBusy()) {}
        }
        while (adjustingOrientation) {}
    }

    private void run(double power) {
        for (DcMotorEx motor : this.allMotors) {
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    public Commands chainReset() {
        for (DcMotorEx motor : this.allMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        return this;
    }

    public int[] getEncoderPositions() {
        int[] encoderPositions = new int[this.allMotors.length];

        for (int i = 0; i < encoderPositions.length; i++) {
            encoderPositions[i] = this.allMotors[i].getCurrentPosition();
        }

        return encoderPositions;
    }

    public Commands forward(double inches, double power) {
        this.setupAllMotors((int) (this.inchesToTicks(inches)));
        this.run(power);

        return this;
    }

    public Commands backward(double inches, double power) {
        return this.forward(-inches, power);
    }

    public Commands strafeLeft(double inches, double power) {
        this.setupLeftRightDiagonal((int) this.inchesToTicks(inches));
        this.setupRightLeftDiagonal((int) this.inchesToTicks(-inches));
        this.run(power);

        return this;
    }

    public Commands strafeRight(double inches, double power) {
        this.setupLeftRightDiagonal((int) this.inchesToTicks(-inches));
        this.setupRightLeftDiagonal((int) this.inchesToTicks(inches));
        this.run(power);

        return this;
    }

    public Commands turnRight(int theta) {
        adjustingOrientation = true;
        int startHeading = this.gyro.getIntegratedZValue();
        int targetHeading = startHeading + theta;
        while (targetHeading-this.gyro.getIntegratedZValue() != 0) {
            // set power to error*kP (Possible PID controller usage)
        }
    }

    public Commands gotoEncoderPosition(int[] encoderPositions, double power) {
        for (int i = 0; i < this.allMotors.length; i++) {
            this.allMotors[i].setTargetPosition(encoderPositions[i]);
        }
        this.run(power);

        return this;
    }
}
