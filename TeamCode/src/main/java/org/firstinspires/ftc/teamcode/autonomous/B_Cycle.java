package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BarcodeDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Commands;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.RobotArm;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous
public class B_Cycle extends LinearOpMode {

    final double CYCLES_NUMBER = 2;

    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LB;
    DcMotorEx ArmMotor;
    DcMotorEx Intake;
    Servo Twist;
    CRServo horizontalServo;
    ColorSensor BoxSensor;
    OpenCvWebcam webcam;

    ModernRoboticsI2cGyro armGyro;

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

    public enum ArmStates {
        REST, LEVEL_1, LEVEL_2, LEVEL_3
    }

    ArmStates currentArmState;
    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;

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

    public double getArmTargetDegrees(ArmStates currentArmStates) {
        double targetDegrees = 0D;
        switch (currentArmStates) {
            case REST:
                targetDegrees = 0D;
                break;
            case LEVEL_1:
                targetDegrees = 150D;
                break;
            case LEVEL_2:
                targetDegrees = 200D;
                break;
            case LEVEL_3:
                targetDegrees = 220D;
                break;
        }

        return targetDegrees;
    }

    public void run() {
        while (opModeIsActive()) {
            double targetArmDegrees = getArmTargetDegrees(currentArmState);
            double power = 0;
            ArmMotor.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {


        armGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "ArmGyro");

        telemetry.log().add("Gyros Calibrating. Do Not Move!");
        armGyro.calibrate();

        while (!isStopRequested() && armGyro.isCalibrating())  {
            telemetry.addData("calibrating, ", "%f seconds passed");
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Done calibrating");
        telemetry.update();

        LF = initMotor(
                "LF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Intake = initMotor(
                "Intake",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Twist = hardwareMap.servo.get("Twisty");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

        Servo leftOdometryServo = hardwareMap.servo.get("LeftOdometryServo");
        Servo rightOdometryServo = hardwareMap.servo.get("RightOdometryServo");
        Servo frontOdometryServo = hardwareMap.servo.get("FrontOdometryServo");

        horizontalServo = hardwareMap.crservo.get("TapeHorizontialOrientation   ");
        horizontalServo.setPower(0);

        double LEFT_POSITION = 0.85D;
        double RIGHT_POSITION = 0.1D;
        double FRONT_POSITION = 0.68D;

        leftOdometryServo.setPosition(LEFT_POSITION);
        rightOdometryServo.setPosition(RIGHT_POSITION);
        frontOdometryServo.setPosition(FRONT_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        BarcodeDetector detector = new BarcodeDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        drive.setPoseEstimate(new Pose2d(0, 0, -180));

//        Trajectory linearToHub = drive.trajectoryBuilder(new Pose2d(0, 0, -180))
//                .lineToConstantHeading(new Vector2d(-18, -26))
//                .build();
//        TrajectorySequence hubToBlocks = drive.trajectorySequenceBuilder(linearToHub.end())
//                .lineToLinearHeading(new Pose2d(0, 10, -90))
//                .lineTo(new Vector2d(15, 0))
//                .build();

        RobotArm arm;

        AtomicInteger currentTargetHeading = new AtomicInteger(0);

        drive.setPoseEstimate(new Pose2d(13, 65, Math.toRadians(90)));


        TrajectorySequence startToHub = drive.trajectorySequenceBuilder(new Pose2d(13, 65, Math.toRadians(90)))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-11, 43, Math.toRadians(90)), Math.toRadians(200))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(100);
                    Twist.setPosition(0.8D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .build();

        TrajectorySequence hubToBlocks = drive.trajectorySequenceBuilder(startToHub.end())
                .splineToSplineHeading(new Pose2d(40, 75, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.5);
                })
                .build();

        TrajectorySequence blocksToHub = drive.trajectorySequenceBuilder(hubToBlocks.end())
                .setReversed(true)
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-11, 43, Math.toRadians(90)), Math.toRadians(270))
                .setReversed(false)
                .build();

        TrajectorySequence getBlocks = drive.trajectorySequenceBuilder(blocksToHub.end())
                .lineToLinearHeading(new Pose2d(65, 65, Math.toRadians(10)))
                .build();



        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(13, 65, Math.toRadians(90)))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, 46, Math.toRadians(90)), Math.toRadians(200))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(40, 70, Math.toRadians(0)), Math.toRadians(0))
                .forward(6, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                //.lineToLinearHeading(new Pose2d(40, 70))
                .waitSeconds(0.1)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, 46, Math.toRadians(90)), Math.toRadians(270), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(40, 70, Math.toRadians(0)), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 16))
                .forward(8, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(43, Math.toRadians(180), 16))
                .lineToLinearHeading(new Pose2d(40, 70))
                .waitSeconds(0.3)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, 46, Math.toRadians(90)), Math.toRadians(270), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(0);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(40, 70, Math.toRadians(0)), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 16))
                .forward(13, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-0.8);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(43, Math.toRadians(180), 16))
                .lineToLinearHeading(new Pose2d(40, 70))
                .addDisplacementMarker(() -> {
                    Intake.setPower(-1);
                })
                .build();



        waitForStart();
        BarcodeDetector.Location barcode = detector.getLocation();
        webcam.stopStreaming();
        switch (barcode) {
            case LEFT:
                telemetry.addData("location: ", "left");
                break;
            case RIGHT:
                telemetry.addData("location: ", "right");
                break;
            case MIDDLE:
                telemetry.addData("location: ", "middle");
        }
        telemetry.update();

        if(isStopRequested()) return;

        /*
        drive.followTrajectorySequenceAsync(startToHub);

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();

            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }

            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading));

        }

        sleep(100);
        Twist.setPosition(0.8D);
        sleep(300);
        Twist.setPosition(0.48D);
        sleep(100);

        for (int i = 0; i < CYCLES_NUMBER; i++) {
            drive.followTrajectorySequenceAsync(hubToBlocks);

            while (drive.isBusy() && opModeIsActive()) {
                drive.update();

                int heading = armGyro.getHeading();

                if (heading > 300) {
                    heading = 0;
                }

                ArmMotor.setPower(controller.calculate(155, heading));
            }


            drive.followTrajectorySequenceAsync(getBlocks);
            drive.getVelocityConstraint(10, Math.toRadians(10), 14);

            while (drive.isBusy() && opModeIsActive()) {
                drive.update();
            }

            sleep(100);

            Trajectory currentToBlocks = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(40, 75))
                    .build();

            drive.followTrajectory(currentToBlocks);

            drive.followTrajectorySequenceAsync(blocksToHub);

            while (drive.isBusy() && opModeIsActive()) {
                drive.update();

                int heading = armGyro.getHeading();

                if (heading > 300) {
                    heading = 0;
                }

                ArmMotor.setPower(controller.calculate(0, heading));
            }

            sleep(100);
            Twist.setPosition(0.8D);
            sleep(300);
            Twist.setPosition(0.48D);
            sleep(100);
        }

        drive.followTrajectorySequence(hubToBlocks);

         */

        drive.followTrajectorySequenceAsync(test);

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();

            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }

            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading));
        }

//        drive.followTrajectory(linearToHub);
//        // Put Block
//        drive.followTrajectorySequence(hubToBlocks);
//        // Get Block
//        drive.followTrajectory(linearToHub);
    }
}
