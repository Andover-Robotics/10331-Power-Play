package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Fourbar extends SubsystemBase{
    public MotorEx fourBar;





 public Fourbar (OpMode opMode)
    {
        fourBar = new MotorEx(opMode.hardwareMap, "fourBar", Motor.GoBILDA.RPM_312);
        fourBar.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourBar.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }




    public void up(double power)
    {
       fourBar.set(power);
    }


    public void down(double power)
    {
        fourBar.set(-power);
    }

    public void toMid()
    {
        fourBar.setRunMode(Motor.RunMode.PositionControl);
        fourBar.setPositionTolerance(5);
        fourBar.setTargetPosition(1300);//2783//5591
        while(!fourBar.atTargetPosition()){
            fourBar.set(0.8);
        }
        fourBar.stopMotor();
        fourBar.setRunMode(Motor.RunMode.RawPower);
    }

    public void toLow()
    {
        fourBar.setRunMode(Motor.RunMode.PositionControl);
        fourBar.setPositionTolerance(5);
        fourBar.setTargetPosition(-680);//2783//5591
        while(!fourBar.atTargetPosition()){
            fourBar.set(0.8);
        }
        fourBar.stopMotor();
        fourBar.setRunMode(Motor.RunMode.RawPower);
    }

    public void toGround()
    {
        fourBar.setRunMode(Motor.RunMode.PositionControl);
        fourBar.setPositionTolerance(5);
        fourBar.setTargetPosition(-2800);//2783//5591
        while(!fourBar.atTargetPosition()){
            fourBar.set(0.8);
        }
        fourBar.stopMotor();
        fourBar.setRunMode(Motor.RunMode.RawPower);
    }

    public void toRetrieval()
    {
        fourBar.setRunMode(Motor.RunMode.PositionControl);
        fourBar.setPositionTolerance(5);
        fourBar.setTargetPosition(-3200);//2783//5591
        while(!fourBar.atTargetPosition()){
            fourBar.set(0.8);
        }
        fourBar.stopMotor();
        fourBar.setRunMode(Motor.RunMode.RawPower);
    }

    public void stop()
    {
        fourBar.stopMotor();
    }

}

