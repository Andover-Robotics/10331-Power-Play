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
        //rgb to hsv, rgb isn't great for object detection most of the time

        //Scalar lowHSV, highHSV; // lower and upper range of yellow
        lowHSV = new Scalar(23, 50, 70);
        //highHSV = new Scalar(32, 255, 255);

        lowHSV = new Scalar(275,61,43);
        highHSV = new Scalar(277,45,85);
        Scalar positionOne = new Scalar(255,102,255);   //find rgb colors used
        Scalar positionTwo = new Scalar(0,204,102);
        Scalar positionThree = new Scalar(102,0,204);
        //thresholding
        //Core.inRange(mat, lowHSV, highHSV, mat);

        //Mat one = mat.submat(ONE_ROI);
        //Mat two = mat.submat(TWO_ROI);
        //Mat three = mat.submat(THREE_ROI);

        //change the colors to be different for each
        //double oneVal = Core.sumElems(one).val[0] / ONE_ROI.area() / 255;
        //double twoVal = Core.sumElems(two).val[0] / TWO_ROI.area() / 255;
        //double threeVal = Core.sumElems(three).val[0] / THREE_ROI.area() / 255;

        //one.release();
        //two.release();
        //three.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(one).val[0]);
        telemetry.addData("Mid raw value", (int) Core.sumElems(two).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(three).val[0]);
        telemetry.addData("Left percentage", Math.round(oneVal * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(twoVal * 100) + "%");
        telemetry.addData("Right percentage", Math.round(threeVal * 100) + "%");

        boolean positionOne = oneVal > PERCENT_COLOR_THRESHOLD;
        boolean positionTwo = twoVal > PERCENT_COLOR_THRESHOLD;
        boolean positionThree = threeVal > PERCENT_COLOR_THRESHOLD;

        if (positionOne) {
            position = Position.ONE;
            telemetry.addData("Parking position: ", "1");
        } else if (positionTwo) {
            position = Position.TWO;
            telemetry.addData("Parking position: ", "2");
        } else if (positionThree) {
            position = Position.THREE;
            telemetry.addData("Parking position: ", "3");
        } else {
            position = Position.NOT_FOUND;
            telemetry.addData("Parking position: ", "not found");
        }
        telemetry.update();

        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        //Scalar colorOne = new Scalar(255, 0, 0);
        //Scalar colorTwo = new Scalar(0, 255, 0);
        //Scalar colorThree = new Scalar();

        Imgproc.rectangle(mat, ONE_ROI, position == Position.ONE ? color:colorNoDuck);
        Imgproc.rectangle(mat, TWO_ROI, position == Position.TWO ? colorDuck:colorNoDuck);
        Imgproc.rectangle(mat, THREE_ROI, position == Position.THREE ? colorDuck:colorNoDuck);

        return mat;


    }




     // returns the location of the duck on the barcode, either LEFT, MIDDLE, RIGHT, or NOT_FOUND
     //@return position;

    public Position getPosition(); {
        return position;
    }
}