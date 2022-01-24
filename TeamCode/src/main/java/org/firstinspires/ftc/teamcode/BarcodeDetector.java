package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 400),
            new Point(160, 640));
    static final Rect MID_ROI = new Rect(
            new Point(160, 400),
            new Point(320, 640));
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public BarcodeDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(95, 100, 20); // lower bound HSV for blue
        Scalar highHSV = new Scalar(125, 255, 255); // higher bound HSV for blue

//        Scalar lowHSV = new Scalar(0, 100, 20); // lower bound HSV for red
//        Scalar highHSV = new Scalar(20, 255, 255);// upper bound HSV for red

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;

        left.release();
        mid.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Mid raw value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");

        boolean barcodeLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean barcodeMid = midValue > PERCENT_COLOR_THRESHOLD;

        if (barcodeLeft) {
            location = Location.LEFT;
            telemetry.addData("Barcode Location", "left" +
                    "");
        } else if (barcodeMid) {
            location = Location.MIDDLE;
            telemetry.addData("Barcode Location", "middle");
        } else {
            location = Location.RIGHT;
            telemetry.addData("Barcode Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notBarcode = new Scalar(255, 0, 0);
        Scalar barcode = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? barcode : notBarcode);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MIDDLE ? barcode : notBarcode);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
