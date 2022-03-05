package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Slides {

    DcMotorEx slidesMotor;
    DcMotorEx encoderMotor;

    int targetEncoderPosition = 0;

    boolean reachedPosition = false;

    public Slides(DcMotorEx slidesMotor, DcMotorEx encoderMotor) {
        this.slidesMotor = slidesMotor;
        this.encoderMotor = encoderMotor;
    }

    public void setTargetEncoderPosition(int newTargetEncoderPosition) {
        this.targetEncoderPosition = newTargetEncoderPosition;
    }

    public void update() {
        if (this.encoderMotor.getCurrentPosition() > targetEncoderPosition) {
            slidesMotor.setPower(-0.7);
            this.reachedPosition = false;
        }

        if (this.encoderMotor.getCurrentPosition() < targetEncoderPosition) {
            this.reachedPosition = true;
        }
    }
}
