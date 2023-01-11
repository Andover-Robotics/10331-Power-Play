package org.firstinspires.ftc.teamcode.b_hardware.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase{
        private Servo claw;
//
    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        closeClaw();
    }
//    // TODO: ask Sasha about how claw works
    public void closeClaw(){
        claw.setPosition(0);
    }
//
    public void openClaw(){
        claw.setPosition(0.15);}


}
