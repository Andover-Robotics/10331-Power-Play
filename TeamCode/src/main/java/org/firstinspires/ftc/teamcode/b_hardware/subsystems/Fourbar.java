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

    }

    public void runToHigh(){
        left.setRunMode(Motor.RunMode.PositionControl);
        left.setPositionTolerance(5);
        left.setTargetPosition(1205);//TODO: set position
        while(!left.atTargetPosition()){
            left.set(0.8);//TODO: set position
        }
        left.stopMotor();
        left.setRunMode(Motor.RunMode.RawPower);


        right.setRunMode(Motor.RunMode.PositionControl);
        right.setPositionTolerance(5);
        right.setTargetPosition(1205);//TODO: set position
        while(!right.atTargetPosition()){
            right.set(0.8);//TODO: set position
        }
        right.stopMotor();
        right.setRunMode(Motor.RunMode.RawPower);


    }

    public void runToMid(){
        left.setRunMode(Motor.RunMode.PositionControl);
        left.setPositionTolerance(5);
        left.setTargetPosition(1205);//TODO: set position
        while(!left.atTargetPosition()){
            left.set(0.8);//TODO: set position
        }
        left.stopMotor();
        left.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setRunMode(Motor.RunMode.RawPower);






        right.setRunMode(Motor.RunMode.PositionControl);
        right.setPositionTolerance(5);
        right.setTargetPosition(1205);//TODO: set position
        while(!right.atTargetPosition()){
            right.set(0.8);//TODO: set position
        }
        right.stopMotor();
        right.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setRunMode(Motor.RunMode.RawPower);


    }

    public void runToLow(){


        left.setRunMode(Motor.RunMode.PositionControl);
        left.setPositionTolerance(5);
        left.setTargetPosition(1205);//TODO: set position
        while(!left.atTargetPosition()){
            left.set(0.8);//TODO: set position
        }
        left.stopMotor();
        left.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setRunMode(Motor.RunMode.RawPower);





        right.setRunMode(Motor.RunMode.PositionControl);
        right.setPositionTolerance(5);
        right.setTargetPosition(1205);//TODO: set position
        while(!right.atTargetPosition()){
            right.set(0.8);//TODO: set position
        }
        right.stopMotor();
        right.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setRunMode(Motor.RunMode.RawPower);


    }

    public void restBar(){
        left.setRunMode(Motor.RunMode.PositionControl);
        left.setPositionTolerance(5);
        left.setTargetPosition(0);//TODO: set position
        while(!left.atTargetPosition()){
           left.set(0.8);//TODO: set position
        }
        left.stopMotor();
        left.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setRunMode(Motor.RunMode.RawPower);



        right.setRunMode(Motor.RunMode.PositionControl);
        right.setPositionTolerance(5);
        right.setTargetPosition(0);//TODO: set position
        while(!right.atTargetPosition()){
            right.set(0.8);//TODO: set position
        }
        right.stopMotor();
        right.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setRunMode(Motor.RunMode.RawPower);



    }

    public void runBar(double speed){
        left.set(speed);
        right.set(speed);

    }

    public void stopBar(){
        left.stopMotor();
        right.stopMotor();
    }

    public void reverseBar(double speed)
    {
        left.set(-speed);
        right.set(-speed);
    }


    public void runLeft(double speed)
    {
        left.set(speed*0.5);
    }

    public void runRight(double speed)
    {
        right.set(speed*0.5);
    }


}

