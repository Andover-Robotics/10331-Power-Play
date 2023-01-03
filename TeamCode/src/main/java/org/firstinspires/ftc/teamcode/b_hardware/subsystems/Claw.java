package org.firstinspires.ftc.teamcode.b_hardware.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase{
    private Servo claw;

    //TODO: find positions
    private static double
            open = 0.2,
            closed = 0.1;



    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);


    }

    public void closeClaw(){
        claw.setPosition(closed);
    }

    public void openClaw(){
        claw.setPosition(open);
    }


}
