package org.firstinspires.ftc.teamcode.autonomous.disabled;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
@Deprecated
public class B_CompleteDC extends LinearOpMode {

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

    DcMotor LF;
    DcMotor RF;
    DcMotor RB;
    DcMotor LB;
    DcMotor ArmMotor;
    Servo Twist;
    ColorSensor BoxSensor;

    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;

    OldPIDController movementController = new OldPIDController(
            0.00001,
            0,
            0.001,
            new double[] {
                    0, 0
            },
            537.7,
            0);

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
            0.615D, 0.65D, 0.9D
    };

    private DcMotor initMotor(
            String motorName,
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        return motor;
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
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        /*
        Commands commandUtil = new Commands(
                orientationGyro, RF, LF, RB, LB, telemetry
        );
*/
        waitForStart();

        /*
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
*/
        /*
        commandUtil.forward(12, 0.01).async();
        telemetry.addData("Done with forward", true);
        telemetry.update();
        sleep(1000);
        */

        RF.setTargetPosition(100);
        LF.setTargetPosition(-100);
        LB.setTargetPosition(-100);
        RB.setTargetPosition(-100);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setPower(0.1);
        LF.setPower(0.1);
        LB.setPower(0.1);
        RB.setPower(0.1);
while(LF.isBusy()){
    sleep(10);

}
        /*
        while (opModeIsActive()) {
            long currentSystemTime = System.currentTimeMillis();
            int encoder_LF = LF.getCurrentPosition();
            int encoder_LB = LB.getCurrentPosition();
            int encoder_RF = RF.getCurrentPosition();
            int encoder_RB = RB.getCurrentPosition();
            int encoder_Arm = Math.abs(ArmMotor.getCurrentPosition());
            double power = 0;

            telemetry.addData("LF encoder: ", encoder_LF);
            telemetry.addData("LB encoder: ", encoder_LB);
            telemetry.addData("RF encoder: ", encoder_RF);
            telemetry.addData("RB encoder: ", encoder_RB);
            telemetry.update();
            int rfTarget = 1000;
            int lfTarget = -1000;
            int lbTarget = -1000;
            int rbTarget = -1000;

            RF.setPower(movementController.calculate(rfTarget, RF.getCurrentPosition()));
            LF.setPower(-movementController.calculate(lfTarget, LF.getCurrentPosition()));
            LB.setPower(-movementController.calculate(lbTarget, LB.getCurrentPosition()));
            RB.setPower(-movementController.calculate(rbTarget, RB.getCurrentPosition()));
        }

         */
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