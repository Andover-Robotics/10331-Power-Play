package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo armServo;

    private static double
            low = 0.2,
            medium = 0.1,
            high = 0.15,
            closed = 0.25; //TODO: positions

    public Arm(OpMode opMode) {

        armServo = opMode.hardwareMap.servo.get("armServo");
        armServo.setDirection(Servo.Direction.FORWARD);
    }

    public void lowPosition() {
        armServo.setPosition(low);

    }

    public void mediumPosition() {
        armServo.setPosition(medium);

    }

    public void highPosition() {
        armServo.setPosition(high);

    }

    public void closed() {
        armServo.setPosition(closed);


    }


    public void runArm(double pos)
    {
        armServo.setPosition(pos); // TODO Test
    }

    public void reverseArm (double pos)
    {
        armServo.setPosition(-pos); //TODO test
    }

    public void stopArm()
    {
        armServo.setPosition(0.0); //TODO test
    }
}
