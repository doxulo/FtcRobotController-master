package org.firstinspires.ftc.teamcode.autonomous.disabled;

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
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Commands;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
@Deprecated
public class GhostWheelAuto extends LinearOpMode {

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

    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;
    OldPIDController movementController;

    final double[] LEVEL_ANGLES = new double[] {
            155D,
            190D,
            215D
    };

    final double[] twistPositions = new double[] {
            0.615D, 0D, 0.9D
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



    private void resetVelocity() {
        RF.setVelocity(0);
        RB.setVelocity(0);
        LF.setVelocity(0);
        LB.setVelocity(0);
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

    private void applyPower(double LF_power, double RF_power, double LB_power, double RB_power) {
        LF.setPower(LF_power);
        RF.setPower(RF_power);
        LB.setPower(LB_power);
        RB.setPower(RB_power);
    }

    private void forward(int ticks) {
        int RBCurrentPosition = RB.getCurrentPosition();
        int LFCurrentPosition = LF.getCurrentPosition();
        int LBCurrentPosition = LB.getCurrentPosition();
        int RFCurrentPosition = RF.getCurrentPosition();

        int lf = ticks+LFCurrentPosition;
        int rb = ticks+RBCurrentPosition;
        int lb = ticks+LBCurrentPosition;
        int rf = ticks-RFCurrentPosition;

        while (
                !isBusy(LFCurrentPosition, lf) &&
                        !isBusy(RBCurrentPosition, rb) &&
                        !isBusy(LBCurrentPosition, lb) &&
                        !isBusy(RFCurrentPosition, rf)
        ) {

            LFCurrentPosition = LF.getCurrentPosition();
            RBCurrentPosition = RB.getCurrentPosition();
            LBCurrentPosition = LB.getCurrentPosition();
            RFCurrentPosition = RF.getCurrentPosition();

            RF.setPower(movementController.calculate(Math.abs(rf), Math.abs(RFCurrentPosition)));
            LF.setPower(movementController.calculate(Math.abs(lf), Math.abs(LFCurrentPosition)));
            RB.setPower(movementController.calculate(Math.abs(rb), Math.abs(RBCurrentPosition)));
            LB.setPower(movementController.calculate(Math.abs(lb), Math.abs(LBCurrentPosition)));

        }
    }
    private void strafe(int ticks) {
        int RBCurrentPosition = RB.getCurrentPosition();
        int LFCurrentPosition = LF.getCurrentPosition();
        int LBCurrentPosition = LB.getCurrentPosition();
        int RFCurrentPosition = RF.getCurrentPosition();

        int lf = ticks-LFCurrentPosition;
        int rb = ticks-RBCurrentPosition;
        int lb = ticks+LBCurrentPosition;
        int rf = ticks-RFCurrentPosition;

        while (
                !isBusy(LFCurrentPosition, lf) &&
                        !isBusy(RBCurrentPosition, rb) &&
                        !isBusy(LBCurrentPosition, lb) &&
                        !isBusy(RFCurrentPosition, rf)
        ) {

            LFCurrentPosition = LF.getCurrentPosition();
            RBCurrentPosition = RB.getCurrentPosition();
            LBCurrentPosition = LB.getCurrentPosition();
            RFCurrentPosition = RF.getCurrentPosition();

            RF.setPower(movementController.calculate(Math.abs(rf), Math.abs(RFCurrentPosition)));
            LF.setPower(movementController.calculate(Math.abs(lf), Math.abs(LFCurrentPosition)));
            RB.setPower(movementController.calculate(Math.abs(rb), Math.abs(RBCurrentPosition)));
            LB.setPower(movementController.calculate(Math.abs(lb), Math.abs(LBCurrentPosition)));

        }
    }

    private void turnRight(int theta) {
        int heading =  orientationGyro.getHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        movementController = new OldPIDController(
                0.003,
                0,
                0.1,
                new double[] {
                        0, 0
                },
                537,
                0
        );

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

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        forward(100);
    }
}
