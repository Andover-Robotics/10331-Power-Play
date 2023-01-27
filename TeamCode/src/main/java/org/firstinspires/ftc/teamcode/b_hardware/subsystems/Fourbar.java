package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Fourbar extends SubsystemBase{
    public MotorEx left;
    public MotorEx right;
    private final PIDFController controller;
    public static double p = 0.04, d = 0, f = 0, staticF = 0.1;
    public static double tolerance = 100, powerUp = 0.4;
    public static int mid = 2900, low = 1700, ground = 40, retrieval = 0, inc = 100, dec = 100;

    private int target = 0;




 public Fourbar (OpMode opMode)
    {
        left = new MotorEx(opMode.hardwareMap, "leftFour", Motor.GoBILDA.RPM_312);
     //   left.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        right = new MotorEx(opMode.hardwareMap, "rightFour", Motor.GoBILDA.RPM_312);
        right.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  right.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(p, 0, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);

    }


    public void runTo (int t)
    {
        controller.setSetPoint(t);
        target = t;
    }

    public void up()
    {
        runTo(target+inc);
    }

    public void down()
    {
        runTo(target-dec);
    }

    public void toMid()
    {
        runTo(mid);
    }

    public void toLow()
    {
        runTo(low);
    }

    public void toGround()
    {
        runTo(ground);
    }

    public void toRetrieval()
    {
        runTo(retrieval);
    }

    public void periodic()
    {
        controller.setPIDF(p, 0, d , f);
        if(controller.atSetPoint()){
            left.set(staticF);
            right.set(staticF);
        }else{
            left.set(powerUp * controller.calculate(left.getCurrentPosition()));
            right.set(powerUp * controller.calculate(right.getCurrentPosition()));
        }
    }

}

