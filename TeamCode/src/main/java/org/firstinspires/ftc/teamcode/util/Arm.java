package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {

    public enum ArmTargetPosition {
        LEVEL_0, LEVEL_1, LEVEL_2, LEVEL_3
    }


    DcMotorEx rotationMotor;
    DcMotorEx extensionMotor;

    PIDController controller;
    public ArmTargetPosition targetPosition = ArmTargetPosition.LEVEL_0;

    public static double lowerBound = 0.37;
    public static double higherBound = 0.92;

    public double currentServoPosition = lowerBound;

    public static int level0Position = 1;
    public static int level1Position = 380;
    public static int level2Position = 425;
    public static int level3Position = 500;
    long startExtensionTime = -1;
    long startRetractionTime = -1;

    boolean extendingSlides = false;
    boolean requestRetractSlides = false;
    boolean slidesRetracted = true;
    boolean slidesMoving = false;

    boolean servoChangingPosition = false;

    long lastServoChangePosition = System.currentTimeMillis();

    Servo boxServo;

    int threshold;

    Telemetry telemetry;

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

    public Arm(PIDController controller, DcMotorEx rotationMotor, DcMotorEx extensionMotor, Servo boxServo, int threshold, Telemetry telemetry) {
        this.controller = controller;
        this.rotationMotor = rotationMotor;
        this.extensionMotor = extensionMotor;
        this.threshold = threshold;
        this.boxServo =  boxServo;
        this.telemetry = telemetry;
    }

    private int getCorrectedArmPosition() {
        int currentPosition = this.rotationMotor.getCurrentPosition()/10;

        return Math.max(currentPosition, 0);
    }

    public void setTargetPosition(ArmTargetPosition targetPosition) {
        if (!this.servoChangingPosition && targetPosition != this.targetPosition) {
            if (targetPosition == ArmTargetPosition.LEVEL_0) {
                this.currentServoPosition = lowerBound;
            } else {
                this.currentServoPosition = higherBound;
            }

            if (this.targetPosition != ArmTargetPosition.LEVEL_0) {
                this.servoChangingPosition = true;
            }
        }

        this.targetPosition = targetPosition;

        this.lastServoChangePosition = System.currentTimeMillis();

        this.updateBox();


    }

    public int getTargetPosition() {
        switch (this.targetPosition) {
            case LEVEL_0:
                return level0Position;
            case LEVEL_1:
                return level1Position;
            case LEVEL_2:
                return level2Position;
            case LEVEL_3:
                return level3Position;
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

        if (this.telemetry != null) {
            this.telemetry.addData("Difference: ", this.getTargetPosition() - this.getCorrectedArmPosition());
        }

    }

    private void updateRotation() {
        if (this.servoChangingPosition) {
            if (System.currentTimeMillis() - this.lastServoChangePosition < 500) {
                this.updateBox();
            } else {
                this.servoChangingPosition = false;
            }

        } else if (this.getCorrectedArmPosition() < 20 && this.targetPosition == ArmTargetPosition.LEVEL_0) {
            this.rotationMotor.setPower(0);
        } else {
            this.rotationMotor.setPower(this.controller.calculate(this.getTargetPosition(), this.getCorrectedArmPosition()));
        }

    }

    private void updateBox() {
        this.boxServo.setPosition(this.currentServoPosition);
    }


//    private void extendSlides() {
//        this.extensionMotor.setPower(0.7);
//    }
//
//    private void safeExtend()  {
//        this.extensionMotor.setPower(0.1);
//    }
//
//    private void retractSlides() {
//        this.extensionMotor.setPower(-0.7);
//    }
//
//    private void safeRetract() {
//        this.extensionMotor.setPower(-0.5);
//    }
}
