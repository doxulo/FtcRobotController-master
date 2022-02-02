package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DuckDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public DuckDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.blur(input, mat, new Size(10,10));
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mat, new Scalar(20,100,20), new Scalar(35,255,255), mat);
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 35));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.erode(mat, mat, erodeElement);
        Imgproc.erode(mat, mat , erodeElement);

        Imgproc.dilate(mat, mat, dilateElement);
        Imgproc.dilate(mat, mat, dilateElement);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, mat, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // if any contour exist...
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            // for each contour, display it in yellow
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                Imgproc.drawContours(input, contours, idx, new Scalar(0, 255, 255));
            }
        }
        return input;
    }
}
