package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Commands {
    int FULL_REVOLUTION = 1440;

    BNO055IMU IMU;

    DcMotor RF;
    DcMotor LF;
    DcMotor RB;
    DcMotor LB;

    DcMotor[] allMotors;
    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    double maxPower;

    public Commands(BNO055IMU IMU, DcMotor RF, DcMotor LF, DcMotor RB, DcMotor LB, double maxPower) {
        this.IMU = IMU;
        this.RF = RF;
        this.LF = LF;
        this.RB = RB;
        this.LB = LB;
        this.maxPower = maxPower;
        this.allMotors = new DcMotor[] {
                RF, RB, LF, LB
        };
        this.rightMotors = new DcMotor[] {
                RF, RB
        };
        this.leftMotors = new DcMotor[] {
                LF, LB
        };

        this.initModes();
    }

    public void initModes() {
        for (DcMotor motor : this.allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setTarget(int encoderPosition) {
        this.setTarget(allMotors, encoderPosition);
    }

    public void setTarget(DcMotor[] motors, int encoderPosition) {
        for (DcMotor motor : motors) {
            this.setTarget(motor, encoderPosition);
        }
    }

    public void setTarget(DcMotor motor, int encoderPosition) {
        motor.setTargetPosition(encoderPosition);
    }

    public void runTo() {
        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(this.maxPower);
        }
    }

    public void stafeLeft() {
    }

    public void forward(double revolutions) {
        this.setTarget((int) (revolutions*537.7));
        this.runTo();
    }

    public void backward(double revolutions) {
        this.forward(-revolutions);
    }

    public void right() {
        this.setTarget(rightMotors, (int) (-FULL_REVOLUTION*0.5));
        this.setTarget(leftMotors, (int) (FULL_REVOLUTION*0.5));
    }
}
