package org.firstinspires.ftc.teamcode.b_hardware.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase{
    private Servo leftClaw;
    private Servo rightClaw;

    //TODO: find positions
    private static double
            leftOpen = 0.2,
            leftClosed = 0.1,
            rightOpen = 0.15,
            rightClosed = 0.25;


    public Claw(OpMode opMode){
        leftClaw = opMode.hardwareMap.servo.get("leftClaw");
        leftClaw.setDirection(Servo.Direction.FORWARD);

        rightClaw = opMode.hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.FORWARD);

    }
    // TODO: ask Sasha about how claw works
    public void closeClaw(){
        leftClaw.setPosition(leftClosed);
        rightClaw.setPosition(rightClosed);
    }

    public void openClaw(){
        leftClaw.setPosition(leftOpen);
        rightClaw.setPosition(rightOpen);
    }


}
