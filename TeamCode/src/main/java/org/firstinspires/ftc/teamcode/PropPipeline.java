package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//mat recognizes color
//pont is a pointer on the interface
//scalar is a color
//rect is drawn on screen
public class PropPipeline extends OpenCvPipeline{

    Telemetry Telemetry;

    Mat mat = new Mat();

    public enum Location{
        LEFT,

        RIGHT,

        MIDDLE
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect (
        new Point(60, 70),
        new Point(90, 80));

    static final Rect MIDDLE_ROI = new Rect (
            new Point(80, 100),
            new Point(50, 60));

    static final Rect RIGHT_ROI = new Rect (
            new Point(40, 90),
            new Point(10, 80));

    static double PERCENT_COLOR_THRESHOLD_BLUE = 0.3; //value of confidence within declared rectangles, likely needs to be higher because it is darker

    public PropPipeline(Telemetry t) {
        telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //        Scalar lowHSV = new Scalar(); both red and blue, take the lightest and darkest
        //        Scalar highHSV = new Scalar();

//      Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);



        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / LEFT_ROI.area() / 255;

        left.release();
        middle.release();
        right.release();

//        telemetry.addData("left raw value"); debugging telemetry

        boolean duckLeft = leftValue > PERCENT_COLOR_THRESHOLD_BLUE;
        boolean duckMiddle = middleValue > PERCENT_COLOR_THRESHOLD_BLUE;
        boolean duckRight = rightValue > PERCENT_COLOR_THRESHOLD_BLUE;

        if(duckRight){
            location = Location.RIGHT;
            telemetry.addData("Duck Location", "Right");
        }
        else if(duckMiddle){
            location = Location.MIDDLE;
            telemetry.addData("Duck Location", "Middle");

        }
        else{
            location = Location.LEFT;
            telemetry.addData("Duck Location", "Middle");

        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);//greyscale for the camera

        Scalar noProp = new Scalar(255, 0 ,0); //red
        Scalar yesProp = new Scalar(0, 255, 0); //green

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? yesProp:noProp);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? yesProp:noProp);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? yesProp:noProp);

        return mat;

    }
    public Location getLocation() {
        return location;
    }
}

