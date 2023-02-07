package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeScanning extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Position {
        ONE,
        TWO,
        THREE,
        NOT_FOUND
    }

    private Position position;

    static final int SCREEN_WIDTH = 1280; //TODO find dimensions of phone
    static final int SCREEN_HEIGHT = 720;


    //static final Rect ONE_ROI = new Rect(new Point(0, 0), new Point((double) SCREEN_WIDTH/3, SCREEN_HEIGHT));
    //static final Rect TWO_ROI = new Rect(new Point((double) SCREEN_WIDTH/3, 0), new Point((double) 2*SCREEN_WIDTH/3, SCREEN_HEIGHT));
    //static final Rect THREE_ROI = new Rect(new Point((double) 2*SCREEN_WIDTH/3, 0), new Point(SCREEN_WIDTH, SCREEN_HEIGHT));

    static final double PERCENT_COLOR_THRESHOLD = 0.0075; //TODO determine percent color threshold


    public ConeScanning(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //test for high and low values
        Scalar positionOneHigh = new Scalar(209,0,0);   // Red: hsv(0,100,82)
        Scalar positionOneLow = new Scalar(171,3,3);

        Scalar positionTwoHigh = new Scalar(255,255,0);     //Yellow hsv(60,100,100)
        Scalar positionTwoLow = new Scalar(173,173,2);

        Scalar positionThreeHigh = new Scalar(23,216,255);     //Blue  hsv(190,91,100)
        Scalar positionThreeLow = new Scalar(18,139,163);

        Core.inRange(mat, positionOneHigh, positionOneLow, mat);
        Core.inRange(mat, positionTwoHigh,positionTwoLow, mat);
        Core.inRange(mat, positionThreeHigh,positionThreeLow, mat);



        //rgb to hsv, rgb isn't great for object detection most of the time

        //Scalar lowHSV, highHSV; // lower and upper range of yellow
        //lowHSV = new Scalar(23, 50, 70);
        //highHSV = new Scalar(32, 255, 255);

        //lowHSV = new Scalar(275,61,43);
        //highHSV = new Scalar(277,45,85);
        //Scalar positionOne = new Scalar(255, 102, 255);
        //Scalar positionTwo = new Scalar(0, 204, 102);
        //Scalar positionThree = new Scalar(102, 0, 204);

        //thresholding
        //Core.inRange(mat, lowHSV, highHSV, mat);

        Mat cone = mat;        //Mat three = mat.submat(THREE_ROI);


        double oneVal = Core.sumElems(cone).val[0] / 255;
        double twoVal = Core.sumElems(cone).val[0] / 255;
        double threeVal = Core.sumElems(cone).val[0] / 255;



        cone.release();

        //two.release();
        //three.release();

        telemetry.addData("First raw value", (int) Core.sumElems(cone).val[0]);
        telemetry.addData("Second raw value", (int) Core.sumElems(cone).val[0]);
        telemetry.addData("Third raw value", (int) Core.sumElems(cone).val[0]);
        telemetry.addData("First percentage", Math.round(oneVal * 100) + "%");
        telemetry.addData("Second percentage", Math.round(twoVal * 100) + "%");
        telemetry.addData("Third percentage", Math.round(threeVal * 100) + "%");

        boolean colorOne = oneVal > PERCENT_COLOR_THRESHOLD;
        boolean colorTwo = twoVal > PERCENT_COLOR_THRESHOLD;
        boolean colorThree = threeVal > PERCENT_COLOR_THRESHOLD;

        if (colorOne) {
            position = Position.ONE;
            telemetry.addData("Parking position: ", "1");
        } else if (colorTwo) {
            position = Position.TWO;
            telemetry.addData("Parking position: ", "2");
        } else if (colorThree) {
            position = Position.THREE;
            telemetry.addData("Parking position: ", "3");
        } else {
            position = Position.NOT_FOUND;
            telemetry.addData("Parking position: ", "not found");
        }
        telemetry.update();


        return mat;


    }

    // returns the location of the duck on the barcode, either LEFT, MIDDLE, RIGHT, or NOT_FOUND
    //@return position;
    public Position getPosition() {
        return position;
    }
}
