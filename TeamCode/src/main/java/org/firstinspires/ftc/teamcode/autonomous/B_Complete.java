package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.util.Map;
import org.firstinspires.ftc.teamcode.util.PIDCommands;
import org.firstinspires.ftc.teamcode.util.PIDController;
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
    Servo Twist;
    ColorSensor BoxSensor;

    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;
    PIDController movementController;

    PIDController controller = new PIDController(
            0.003,
            0.0000001,
            0.01,
            new double[] {
                    0.05, -0.10
            },
            360,
            0);

    PIDController downController = new PIDController(
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
        /*if (calculatedVelocity < 0) {
            calculatedVelocity = Math.max(calculatedVelocity, -MAX_VELOCITY);
        } else {
            calculatedVelocity = Math.min(calculatedVelocity, MAX_VELOCITY);
        }*/
        telemetry.addData("Power: ", calculatedVelocity);
        motor.setVelocity(
                calculatedVelocity/100
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
            return 1;
        } else if (xPosition > -0.35 && xPosition < 0.2) {
            return 2;
        } else {
            return 3;
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
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Twist = hardwareMap.servo.get("Twist");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

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


        Commands commandUtil = new Commands(
                orientationGyro, RF, LF, RB, LB, telemetry
        );

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();

        int level = -1;

        for (int i = 0; i < MAX_TRIES; i++) {
            level = getLevel();
            if (level != -1) {
                break;
            }
            sleep(100);
        }

        level = level == -1 ? 0 : level;


        telemetry.addData("Level: ", level);
        telemetry.addData("Busy: ", LF.isBusy());
        telemetry.update();

        /*
        commandUtil.forward(12, 0.01).async();
        telemetry.addData("Done with forward", true);
        telemetry.update();
        sleep(1000);


         */

        movementController = new PIDController(
                0.1,
                0,
                0,
                new double[] {0, 0},
                0,
                0);
        int RFCurrentPosition = RF.getCurrentPosition();
        int LFCurrentPosition = LF.getCurrentPosition();
        int RBCurrentPosition = RB.getCurrentPosition();
        int LBCurrentPosition = LB.getCurrentPosition();

        int RFTarget = (int) (RFCurrentPosition + 1000);
        int LFTarget = (int) (LFCurrentPosition - 1000);
        int RBTarget = (int) (RBCurrentPosition - 1000);
        int LBTarget = (int) (LBCurrentPosition - 1000);

        movementController.resume();
        while (
                !isBusy(RFCurrentPosition, RFTarget) &&
                        !isBusy(LFCurrentPosition, LFTarget) &&
                        !isBusy(RBCurrentPosition, RBTarget) &&
                        !isBusy(LBCurrentPosition, LBTarget)
        ) {
            telemetry.addLine(String.format("%s, %d, %d", RF.getDeviceName(), RF.getCurrentPosition(), RFTarget));
            telemetry.addLine(String.format("%s, %d, %d", LF.getDeviceName(), LF.getCurrentPosition(), RFTarget));
            telemetry.addLine(String.format("%s, %d, %d", LB.getDeviceName(), LB.getCurrentPosition(), RFTarget));
            telemetry.addLine(String.format("%s, %d, %d", RB.getDeviceName(), RB.getCurrentPosition(), RFTarget));

            telemetry.addLine(String.format("%s, Velocity: %f, %f", RF.getDeviceName(), RF.getVelocity(), RF.getPower()));
            telemetry.addLine(String.format("%s, Velocity: %f, %f", LF.getDeviceName(), LF.getVelocity(), LF.getPower()));
            telemetry.addLine(String.format("%s, Velocity: %f, %f", RB.getDeviceName(), RB.getVelocity(), RB.getPower()));
            telemetry.addLine(String.format("%s, Velocity: %f, %f", LB.getDeviceName(), LB.getVelocity(), LB.getPower()));
            telemetry.update();
            
            setVelocity(RF, RFCurrentPosition, RFTarget);
            setVelocity(LF, LFCurrentPosition, LFTarget);
            setVelocity(RB, RBCurrentPosition, RBTarget);
            setVelocity(LB, LBCurrentPosition, LBTarget);
            RFCurrentPosition = RF.getCurrentPosition();
            LFCurrentPosition = LF.getCurrentPosition();
            RBCurrentPosition = RB.getCurrentPosition();
            LBCurrentPosition = LB.getCurrentPosition();
        }
        resetVelocity();
        movementController.pauseAndReset();
        /*
        commandUtil.backward(12, 0.1).async();
        telemetry.addData("Done with backward", true);
        telemetry.update();
        commandUtil.forward(6, 0.1).async();
        telemetry.addData("Done with forward", true);
        telemetry.update();
        commandUtil.strafeLeft(6, 0.1).async();
        telemetry.addData("Done with strafeLeft", true);
        telemetry.update();
        commandUtil.strafeRight(6, 0.1).async();
        telemetry.addData("Done with strafeRight", true);
        telemetry.update();
         */
        /*
        commandUtil.strafeRight(12, 1).async();
        commandUtil.backward(12, 1).async();

        liftAndPlaceBlockAsync(LEVEL_ANGLES[level], theta);
        controller.pauseAndReset();

        long startTime = elapsedTime.milliseconds()
        while (elapsedTime.milliseconds()-startTime < 1000) {
            ArmMotor.setPower(downController.calculate(0, theta));
        }
        downController.pauseAndReset();
        ArmMotor.setPower(-0.1);

        commandUtil.right(90, 1).async();
        commandUtil.strafeLeft(12, 0.5).async();
        commandUtil.forward(20, 1).async();

        int[] motorPositions = commandUtil.getEncoderPositions();

        while (BoxSensor.red()<85) {
            commandUtil.forward(1, 0.2).async();
        }

        commandUtil.gotoEncoderPosition(motorPositions).async();
        commandUtil.backward(20, 1).async();
        commandUtil.right(90, 1).async();
        commandUtil.forward(10, 1).async();
        liftAndPlaceBlockAsync();

        commandUtil.right(90, 1);
        commandUtil.forward(12, 0.5);
        commandUtil.right(90, 1);
        commandUtil.forward(12, 0.5);
        */

    }
}
