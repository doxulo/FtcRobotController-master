package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotArm {

    DcMotorEx armMotor;

    PIDController controller = new PIDController(
            0.01,
            0.000001,// 0.000001, // TODO: Tune this
            0.05,
            new double[] {
                    0.05, -0.10
            },
            360,
            0,
            new double[] {
                    -0.15, 0.15, 25D
            });

    double targetHeading = 0;
    ModernRoboticsI2cGyro gyro;
    boolean alive = true;

    public RobotArm(DcMotorEx armMotor, ModernRoboticsI2cGyro gyro) {
        this.armMotor = armMotor;
        this.gyro = gyro;
    }

    public void run() {
        while (this.alive) {
            this.updatePower();
        }
    }

    public void updatePower() {
        double power = this.getPower(this.gyro.getHeading());
        this.armMotor.setPower(power);
    }

    public void updateHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getPower(double currentHeading) {
        return controller.calculate(this.targetHeading, currentHeading);
    }
}

