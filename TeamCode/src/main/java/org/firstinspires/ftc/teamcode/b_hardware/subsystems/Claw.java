package org.firstinspires.ftc.teamcode.b_hardware.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase{
    private Servo clawServo;

    public Claw(OpMode opMode){
        clawServo = opMode.hardwareMap.servo.get("clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);

    }
    // TODO: ask Sasha about how claw works
}
