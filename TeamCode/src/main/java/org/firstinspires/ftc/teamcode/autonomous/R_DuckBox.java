package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Map;

@Disabled
public class R_DuckBox extends LinearOpMode {
    Map objectMap = new Map();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor duckWheel = objectMap.get("Duck_Wheel");
        waitForStart();

        Trajectory right = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        drive.followTrajectoryAsync(right);
        duckWheel.setPower(0.56);
        sleep(750);
        duckWheel.setPower(1);
        sleep(500);
        duckWheel.setPower(0);
        drive.followTrajectoryAsync(forward);
    }
}