package org.firstinspires.ftc.teamcode.autonomous.league;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.BarcodeDetector;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.RobotArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(group = "League")
@Disabled
public class R_DuckBox extends LinearOpMode {

    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LB;
    DcMotorEx ArmMotor;
    Servo Twist;
    ColorSensor BoxSensor;
    OpenCvWebcam webcam;
//    OpenCvWebcam duckwebcam;
    DcMotorEx Duck_Wheel2;
    DcMotorEx Duck_Wheel1;
    DcMotorEx Intake;
    CRServo horizontalServo;

    ModernRoboticsI2cGyro armGyro;

    public enum ArmStates {
        REST, LEVEL_1, LEVEL_2, LEVEL_3
    }

    ArmStates currentArmState;
    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;

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

    public double getArmTargetDegrees(int level) {
        double targetDegrees = 0D;
        switch (level) {
            case 0:
                targetDegrees = 0D;
                break;
            case 1:
                targetDegrees = 150D;
                break;
            case 2:
                targetDegrees = 185D;
                break;
            case 3:
                targetDegrees = 205D;
                break;
        }

        return targetDegrees;
    }

    public double getPower(double currentHeading, int targetLevel) {
        return controller.calculate(getArmTargetDegrees(targetLevel), currentHeading);
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

        Intake = initMotor(
                "Intake",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RobotArm arm = new RobotArm(ArmMotor, armGyro);
        // arm.start();

        Twist = hardwareMap.servo.get("Twisty");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

        Servo leftOdometryServo = hardwareMap.servo.get("LeftOdometryServo");
        Servo rightOdometryServo = hardwareMap.servo.get("RightOdometryServo");
        Servo frontOdometryServo = hardwareMap.servo.get("FrontOdometryServo");

        horizontalServo = hardwareMap.crservo.get("TapeHorizontialOrientation   ");
        horizontalServo.setPower(0);

        double LEFT_POSITION = 0.85D;
        double RIGHT_POSITION = 0.1D;
        double FRONT_POSITION = 0.65D;

        double armTargetHeading = 0D;

        leftOdometryServo.setPosition(LEFT_POSITION);
        rightOdometryServo.setPosition(RIGHT_POSITION);
        frontOdometryServo.setPosition(FRONT_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

//        duckwebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);


        // OR...  Do Not Activate the Camera Monitor View
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        BarcodeDetector detector = new BarcodeDetector(telemetry);
//        DuckDetector detector1 = new DuckDetector(telemetry);

        webcam.setPipeline(detector);
//        duckwebcam.setPipeline(detector1);

//        webcam.setMillisecondsPermissionTimeout(2500);
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

//        duckwebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                duckwebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(0, 0, -180));

//        Trajectory linearToHub = drive.trajectoryBuilder(new Pose2d(0, 0, -180))
//                .lineToConstantHeading(new Vector2d(-18, -26))
//                .build();
//        TrajectorySequence hubToBlocks = drive.trajectorySequenceBuilder(linearToHub.end())
//                .lineToLinearHeading(new Pose2d(0, 10, -90))
//                .lineTo(new Vector2d(15, 0))
//                .build();

        AtomicInteger currentTargetHeading = new AtomicInteger(0);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

        TrajectorySequence lv1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-29, 10, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0.55);
                    Duck_Wheel2.setPower(-0.55);
                })
                .turn(Math.toRadians(-5))
                .strafeRight(3)
                .waitSeconds(3)
                .strafeLeft(10)
                .turn(Math.toRadians(-85))
                .back(35)
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0);
                    Duck_Wheel2.setPower(0);
                    currentTargetHeading.set(155);
                })
                .turn(Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(-2,38, Math.toRadians(180)))
                .build();

        TrajectorySequence lv2 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-29, 10, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0.55);
                    Duck_Wheel2.setPower(-0.55);
                })
                .strafeRight(6)
                .waitSeconds(3)
                .strafeLeft(10)
                .turn(Math.toRadians(-90))
                .back(35)
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0);
                    Duck_Wheel2.setPower(0);
                    currentTargetHeading.set(190);
                })
                .turn(Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(-7,38, Math.toRadians(180)))
                .build();

        TrajectorySequence lv3 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-29, 10, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0.55);
                    Duck_Wheel2.setPower(-0.55);
                })
                .strafeRight(6)
                .waitSeconds(3)
                .strafeLeft(10)
                .turn(Math.toRadians(-90))
                .back(35)
                .addDisplacementMarker(() -> {
                    Duck_Wheel1.setPower(0);
                    Duck_Wheel2.setPower(0);
                    currentTargetHeading.set(215);
                })
                .turn(Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(-13,38, Math.toRadians(180)))
                .build();

        // 10 high 5 mid 0 low
        TrajectorySequence lv1_warehouse_from_hub = drive.trajectorySequenceBuilder(lv1.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    currentTargetHeading.set(0);
                })
                .lineToConstantHeading(new Vector2d(-20, 42))
                .lineToLinearHeading(new Pose2d(-30, 25.5, Math.toRadians(0)))
                .build();

        TrajectorySequence lv2_warehouse_from_hub = drive.trajectorySequenceBuilder(lv2.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    currentTargetHeading.set(0);
                })
                .lineToConstantHeading(new Vector2d(-20, 42))
                .lineToLinearHeading(new Pose2d(-30, 25.5, Math.toRadians(0)))
                .build();

        TrajectorySequence lv3_warehouse_from_hub = drive.trajectorySequenceBuilder(lv3.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    currentTargetHeading.set(0);
                })
                .lineToConstantHeading(new Vector2d(-20, 42))
                .lineToLinearHeading(new Pose2d(-30, 25.5, Math.toRadians(0)))
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


        int targetLevel = 0;
        Twist.setPosition(0.58D);


        switch (barcode) {
            case LEFT:
                targetLevel = 3;
                drive.followTrajectorySequenceAsync(lv3);
                break;
            case RIGHT:
                targetLevel = 1;
                drive.followTrajectorySequenceAsync(lv1);
                break;
            case MIDDLE:
                targetLevel = 2;
                drive.followTrajectorySequenceAsync(lv2);
        }

        while (drive.isBusy()) {
            drive.update();

            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }
            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading));
        }

        ElapsedTime t = new ElapsedTime();

        while (t.milliseconds() < 100) {
            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }

            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading));
        }

        Intake.setPower(-0.3);

        sleep(200);
        Twist.setPosition(0.8D);
        sleep(250);
        Twist.setPosition(0.58);

        switch (barcode) {
            case LEFT:
                drive.followTrajectorySequenceAsync(lv3_warehouse_from_hub);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(lv1_warehouse_from_hub);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(lv2_warehouse_from_hub);
        }

        while (drive.isBusy()) {
            drive.update();

            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }
            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading)*0.5);
        }
    }
}
