package org.firstinspires.ftc.teamcode.a_opmodes.auto;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.ConeScanning;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name = "Main Autonomous Random Park", group = "Competition")
public class MainRandomPark extends LinearOpMode {//TODO: add reversing for competition

    private Bot bot;

    TemplateDetector.PipelineResult detected;
    double confidence;
    TemplateDetector pipeline;
    boolean performActions = true;
    GamepadEx gamepad;



    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);
        gamepad = new GamepadEx(gamepad1);


        RandomPark paths = new RandomPark(this);
//    pipeline = new TemplateDetector(this);

        //TODO: add initialization here

        //  ie set servo position                             ========================================================================


        //Pipeline stuff

//    while (!isStarted()) {
//      if (isStopRequested())
//        return;
//      // keep getting results from the pipeline
//      pipeline.currentlyDetected()
//          .ifPresent((pair) -> {
//            telemetry.addData("detected", pair.first);
//            telemetry.addData("confidence", pair.second);
//            telemetry.update();
//            detected = pair.first;
//            confidence = pair.second;
//          });
//      if (gamepad1.x) {
//        performActions = false;
//      }
//      if (gamepad.wasJustPressed(Button.Y)) {
//        pipeline.saveImage();
//      }
//    }
//
//    pipeline.currentlyDetected().ifPresent(pair -> {
//      detected = pair.first;
//      confidence = pair.second;
//    });
//
//    if (detected == null)
//      detected = PipelineResult.LEFT;

        detected = TemplateDetector.PipelineResult.RIGHT;

        telemetry.addLine(GlobalConfig.alliance + " is selected alliance");

        telemetry.update();


        waitForStart();

        //List<AutoPathElement> trajectories = paths.getTrajectories (detected);
        //       List<FullSide.AutoPathElement> trajectories = paths.getTrajectories(detected);
        List<RandomPark.AutoPathElement> trajectories = paths.getTrajectories(detected);
//    pipeline.close();


        //Roadrunner stuff

        bot.roadRunner.setPoseEstimate(paths.getStartPose());

        if (isStopRequested())
            return;

        for (RandomPark.AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof RandomPark.AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((RandomPark.AutoPathElement.Path) item).getTrajectory());
            } else if (item instanceof RandomPark.AutoPathElement.Action && performActions) {
                ((RandomPark.AutoPathElement.Action) item).getRunner().invoke();
            }

            if (isStopRequested())
                return;
        }
    }


}