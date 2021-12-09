package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Commands;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class B_Complete extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.1;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    final int MAX_TRIES = 10;

    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LB;
    DcMotorEx ArmMotor;
    DcMotor Duck_Wheel1;
    DcMotor Duck_Wheel2;
    Servo Twist;
    ColorSensor BoxSensor;
    public Servo[] odometryServos = new Servo[3];

    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;
    OldPIDController movementController;

    OldPIDController controller = new OldPIDController(
            0.003,
            0.0000001,
            0.01,
            new double[] {
                    0.05, -0.10
            },
            360,
            0);

    OldPIDController downController = new OldPIDController(
            0.004,
            0,
            0.0075,
            new double[] {
                    0.05, -0.10
            },
            1440,
            0);

    final double[] LEVEL_ANGLES = new double[] {
            155D,
            190D,
            215D
    };

    final double[] twistPositions = new double[] {
            0.615D, 0.7D, 0.9D
    };

    double MAX_VELOCITY = 1200;
    double kP = 0.0001D;

    private DcMotorEx initMotor(
            String motorName,
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        return motor;
    }

    private boolean isBusy(int motorPosition, int target) {
        return Math.abs(motorPosition-target) < 0;
    }

    private void setVelocity(DcMotorEx motor, int currentPosition, int targetPosition) {
        if (movementController.paused) {
            movementController.resume();
        }

        double calculatedVelocity = movementController.calculate((double)targetPosition, (double) currentPosition);
        telemetry.addLine(String.format("Power: %f", calculatedVelocity));
        if (calculatedVelocity < 0) {
            calculatedVelocity = Math.max(calculatedVelocity, -MAX_VELOCITY);
        } else {
            calculatedVelocity = Math.min(calculatedVelocity, MAX_VELOCITY);
        }
        telemetry.addData("Power: ", calculatedVelocity);
        motor.setVelocity(
                calculatedVelocity
        );
    }

    private void resetVelocity() {
        RF.setVelocity(0);
        RB.setVelocity(0);
        LF.setVelocity(0);
        LB.setVelocity(0);
    }

    private void liftAndPlaceBlockAsync(double targetPosition, double theta) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < 1) {
            ArmMotor.setPower(controller.calculate(targetPosition, theta));
        }

        Twist.setPosition(twistPositions[2]);
        sleep(250);
        Twist.setPosition(twistPositions[0]);

        ArmMotor.setPower(controller.getFloat(theta));
    }

    public int getLevel(double xPosition) {
        if (xPosition < -0.35) {
            return 0;
        } else if (xPosition > -0.35 && xPosition < 0.2) {
            return 1;
        } else {
            return 2;
        }
    }

    public int getLevel() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if(detections != null)
        {

            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }

                telemetry.addData("Size: ", detections.size());
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections) {
                    telemetry.addData("xPosition: ", detection.pose.x);
                    telemetry.addData("yPosition: ", detection.pose.y);
                    telemetry.addData("zPosition: ", detection.pose.z);
                    telemetry.update();
                    return getLevel(detection.pose.x);
                }
            }
        }

        return -1;
    }

    private void applyPower(double LF_power, double RF_power, double LB_power, double RB_power) {
        LF.setPower(LF_power);
        RF.setPower(RF_power);
        LB.setPower(LB_power);
        RB.setPower(RB_power);
    }

    @Override
    public void runOpMode() throws InterruptedException {



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        LF = initMotor(
                "LF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Duck_Wheel1 = initMotor(
                "Duck_Wheel1",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Duck_Wheel2 = initMotor(
                "Duck_Wheel2",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Twist = hardwareMap.servo.get("Twist");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");
        odometryServos = new Servo[] {
                hardwareMap.servo.get("LeftOdometryServo"),
                hardwareMap.servo.get("FrontOdometryServo"),
                hardwareMap.servo.get("RightOdometryServo"),
        };
        ElapsedTime timer = new ElapsedTime();

        orientationGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        orientationGyroParsed = (IntegratingGyroscope) orientationGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        orientationGyro.calibrate();

        timer.reset();
        while (!isStopRequested() && orientationGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        double servoOnPosition = 0.36D;
        int level = -1;

        while (!opModeIsActive()) {
            for (int i = 0; i < MAX_TRIES; i++) {
                level = getLevel();
                if (level != -1) {
                    break;
                }
                sleep(100);
            }
        }

        level = level == -1 ? 0 : level;

        for (Servo odometryServo : odometryServos) {
            odometryServo.setPosition(servoOnPosition);
        }

    }
}
