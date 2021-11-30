package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Commands {
    double WHEEL_DIAMETER = 2;
    double WHEEL_CIRCUMFERENCE = 2*WHEEL_DIAMETER*Math.PI;

    int FULL_REVOLUTION_TICKS = 1440;

    double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE/FULL_REVOLUTION_TICKS;

    ModernRoboticsI2cGyro gyro;

    DcMotor RF;
    DcMotor LF;
    DcMotor RB;
    DcMotor LB;

    DcMotor[] allMotors;
    DcMotor[] forwardMotors;
    DcMotor[] backwardMotors;
    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    public Commands(ModernRoboticsI2cGyro gyro, DcMotor RF, DcMotor LF, DcMotor RB, DcMotor LB) {
        this.gyro = gyro;
        this.RF = RF;
        this.LF = LF;
        this.RB = RB;
        this.LB = LB;
        this.allMotors = new DcMotor[] {
                RF, RB, LF, LB
        };
        this.rightMotors = new DcMotor[] {
                RF, RB
        };
        this.leftMotors = new DcMotor[] {
                LF, LB
        };

        /*
        for (int i = 0; i < allMotors.length; i++) {
            switch (allMotors[i].getDirection()) {
                case FORWARD:
                    forwardMotors[i] = allMotors[i];
                    break;
                case REVERSE:
                    backwardMotors[i] = allMotors[i];
                    break;
            }
        }

         */
    }

    private void setupAllMotors(int encoderTicks) {
        for (DcMotor motor : forwardMotors) {
            motor.setTargetPosition(encoderTicks);
        }

        for (DcMotor motor : backwardMotors) {
            motor.setTargetPosition(-encoderTicks);
        }
    }

    private void setupLeftRightDiagonal(int encoderTicks) {
        LF.setTargetPosition(encoderTicks);
        RB.setTargetPosition(encoderTicks);
    }


    private void setupRightLeftDiagonal(int encoderTicks) {
        LB.setTargetPosition(encoderTicks);
        RF.setTargetPosition(encoderTicks);
    }

    public void async() {
        for (DcMotor motor : allMotors) {
            // while (motor.isBusy()) {}
        }
    }

    private void run(double power) {
        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    public Commands chainReset() {
        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return this;
    }

    public Commands forward(double inches, double power) {
        this.setupAllMotors((int) (inches*DISTANCE_PER_TICK));
        this.run(power);

        return this;
    }

    public Commands backward(double inches, double power) {
        return this.forward(-inches, power);
    }

    public Commands strafeLeft(double inches, double power) {
        this.setupLeftRightDiagonal((int) (inches*DISTANCE_PER_TICK));
        this.setupRightLeftDiagonal((int) (-inches*DISTANCE_PER_TICK));
        this.run(power);

        return this;
    }

    public Commands strafeRight(double inches, double power) {
        this.setupLeftRightDiagonal((int) (-inches*DISTANCE_PER_TICK));
        this.setupRightLeftDiagonal((int) (inches*DISTANCE_PER_TICK));
        this.run(power);

        return this;
    }
}
