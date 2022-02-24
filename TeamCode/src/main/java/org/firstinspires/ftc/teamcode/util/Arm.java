package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public enum ArmTargetPosition {
        LEVEL_0, LEVEL_1, LEVEL_2, LEVEL_3
    }


    DcMotorEx rotationMotor;
    DcMotorEx extensionMotor;

    PIDController controller;
    ArmTargetPosition targetPosition;

    double[] bounds = new double[] {0.3, 0.88};
    int[] targetPositions = new int[] {1, 390, 425, 500};
    long startExtensionTime = -1;
    long startRetractionTime = -1;

    boolean extendingSlides = false;
    boolean requestRetractSlides = false;
    boolean slidesRetracted = true;
    boolean slidesMoving = false;

    Servo boxServo;

    int threshold;

    public double lerp(double p0, double p1, double t) {
        return (1-t)*p0 + p1*t;
    }

    public Arm(PIDController controller, DcMotorEx rotationMotor, DcMotorEx extensionMotor, Servo boxServo, int threshold) {
        this.controller = controller;
        this.rotationMotor = rotationMotor;
        this.extensionMotor = extensionMotor;
        this.threshold = threshold;
        this.boxServo =  boxServo;
    }

    private int getCorrectedArmPosition() {
        int currentPosition = this.rotationMotor.getCurrentPosition()/10;

        return Math.max(currentPosition, 0);
    }

    public void setTargetPosition(ArmTargetPosition targetPosition) {

        if (!(targetPosition == this.targetPosition)) {
            if (targetPosition == ArmTargetPosition.LEVEL_3 || targetPosition == ArmTargetPosition.LEVEL_0) {
                if (!this.slidesRetracted) {
                    this.requestRetractSlides = true;
                }
            }
        }

        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        switch (this.targetPosition) {
            case LEVEL_0:
                return targetPositions[0];
            case LEVEL_1:
                return targetPositions[1];
            case LEVEL_2:
                return targetPositions[2];
            case LEVEL_3:
                return targetPositions[3];
        }

        return 0;
    }

    public void update() {
        // Don't move arm while retracting
        if (this.slidesRetracted && !this.requestRetractSlides) {
            this.updateRotation();
        } else {
            this.rotationMotor.setPower(0);
        }

        this.updateExtension();
        this.updateBox();
    }

    private void updateRotation() {
        this.rotationMotor.setPower(this.controller.calculate(this.getTargetPosition(), this.getCorrectedArmPosition()));
    }

    private void updateExtension() {
        if (this.requestRetractSlides) {
            this.retractSlides();
            this.requestRetractSlides = false;
            this.extendingSlides = false;
            this.startRetractionTime = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - this.startRetractionTime > 2000) {
            this.safeRetract();
            this.slidesRetracted = true;
        }

        if (this.targetPosition == ArmTargetPosition.LEVEL_1 || this.targetPosition == ArmTargetPosition.LEVEL_2) {
            if (Math.abs(this.getTargetPosition() - this.getCorrectedArmPosition()) < this.threshold && !this.extendingSlides) {
                this.extendSlides();
                this.startExtensionTime = System.currentTimeMillis();
                this.extendingSlides = true;
                this.slidesRetracted = false;
            } else if (this.extendingSlides && System.currentTimeMillis() - this.startExtensionTime > 2000) {
                this.safeExtend();
            }
        }
    }

    private void updateBox() {
        double position = this.lerp(this.bounds[0], this.bounds[1], Math.min(this.getCorrectedArmPosition()/400, 1));
        this.boxServo.setPosition(position);
    }

    private void extendSlides() {
        this.extensionMotor.setPower(0.5);
    }

    private void safeExtend()  {
        this.extensionMotor.setPower(0.1);
    }

    private void retractSlides() {
        this.extensionMotor.setPower(-0.5);
    }

    private void safeRetract() {
        this.extensionMotor.setPower(-0.1);
    }
}
