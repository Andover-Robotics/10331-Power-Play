package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "Testing TeleOp", group = "Competition")
public class TestTeleOp extends LinearOpMode {//required vars here
    private double driveSpeed = 1;
    private boolean isManual = false;

    private Bot bot;


    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);

        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//        // Retrieve the IMU from the hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
//        imu.initialize(parameters);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepadEx2.getButton(Button.DPAD_UP)){
                bot.claw.openClaw();
            }
            else{
                bot.claw.closeClaw();
            }



            if(gamepadEx2.getButton(Button.Y)) {
              bot.fourBar.toMid();
            }else if(gamepadEx2.getButton(Button.B)) {
              bot.fourBar.toLow();
            }else if(gamepadEx2.getButton(Button.A)){
              bot.fourBar.toGround();
            }else if(gamepadEx2.getButton(Button.X)) {
              bot.fourBar.toRetrieval();
            }


            if (gamepadEx2.getButton(Button.RIGHT_BUMPER)) {
                bot.fourBar.down();
            }

            if (gamepadEx2.getButton(Button.LEFT_BUMPER)) {
                bot.fourBar.up();
            }

            bot.fourBar.periodic();

            driveSpeed = 1;
            boolean slowMode = false;
            double slowModeSpeed = 0.2;


            if (gamepadEx1.getButton(Button.LEFT_BUMPER)) {
                slowMode = true;
            }

            if (slowMode == true) {
                driveSpeed = slowModeSpeed;
            }
            bot.fixMotors();
            bot.drive(gamepadEx1.getLeftX() * driveSpeed, -gamepadEx1.getLeftY() * driveSpeed, gamepadEx1.getRightX() * driveSpeed/1.5);

        }
    }

}

