package org.firstinspires.ftc.teamcode.b_hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Fourbar;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;

public class Bot {
  // in TeleOp and Autonomous we should be able to call "new Bot(this)"
  // bot.intake.run(), bot.shooter.spinUp
  public static Bot instance;


  //TODO: Declare subsystems here



  //required subsystems
  public final MecanumDrive drive;
  public final RRMecanumDrive roadRunner;
  public final BNO055IMU imu;
  private MotorEx fl, fr, bl, br;

  public final Fourbar fourBar;
  public OpMode opMode;
  public final Claw claw;
  public final BNO055IMU imu0;
  public final BNO055IMU imu1;


    /** Get the current Bot instance from somewhere other than an OpMode */
  public static Bot getInstance() {
    if (instance == null) {
      throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
    }
    return instance;
  }

  public static Bot getInstance(OpMode opMode) {
    if (instance == null) {
      return instance = new Bot(opMode);
    }
    instance.opMode = opMode;
    return instance;
  }

  public void reset(){
    //TODO: add reset code here
  }

  private Bot (OpMode opMode){
    this.opMode = opMode;
    enableAutoBulkRead();
    try {
//      this.hubs = Pair.create(opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"), // TODO: check if revextensions2 works with sdk7.0 and control hubs
//          opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"));
    } catch (Exception e) {
      // Avoid catastrophic errors if RevExtensions don't behave as expected. Limited trust of stability
      e.printStackTrace();
    }

    //TODO: initialize subsystems
    //example
//    this.templateSubsystem = new TemplateSubsystem(opMode);
//    this.carousel = new Carousel(opMode);
//    this.intake = new Intake(opMode);
    this.fourBar = new Fourbar(opMode);
    this.claw = new Claw(opMode);
//    this.arm = new Arm(opMode);


    fl = new MotorEx(opMode.hardwareMap, "motorFL");
    fr = new MotorEx(opMode.hardwareMap, "motorFR");
    bl = new MotorEx(opMode.hardwareMap, "motorBL");
    br = new MotorEx(opMode.hardwareMap, "motorBR");

    //required subsystems
    this.drive = new MecanumDrive(fl, fr, bl, br);

    //required subsystems
//    this.drive = new MecanumDrive(false,
//        new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL),
//        new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR),
//        new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL),
//        new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR));
    this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);
//    this.cosmetics = new Cosmetics(opMode);
    imu = roadRunner.imu;
    imu0 = roadRunner.imu;
    imu1 = (roadRunner.imu2 != null) ? roadRunner.imu2 : null;
  }

//  private void initializeImu() {
//    final Parameters params = new Parameters();
//    params.angleUnit = AngleUnit.RADIANS;
//    imu.initialize(params);
//  }

  public void fixMotors(){
    drive.setRightSideInverted(true);

    fl.setInverted(false);
    fr.setInverted(true);
    bl.setInverted(false);
    br.setInverted(true);

    fl.setRunMode(MotorEx.RunMode.RawPower);
    fr.setRunMode(MotorEx.RunMode.RawPower);
    bl.setRunMode(MotorEx.RunMode.RawPower);
    br.setRunMode(MotorEx.RunMode.RawPower);

    fl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
  }

  public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed){
    double speeds[] = {
            forwardBackSpeed-strafeSpeed-turnSpeed,
            forwardBackSpeed+strafeSpeed+turnSpeed,
            forwardBackSpeed+strafeSpeed-turnSpeed,
            forwardBackSpeed-strafeSpeed+turnSpeed
    };
    double maxSpeed = 0;
    for(int i = 0; i < 4; i++){
      maxSpeed = Math.max(maxSpeed, speeds[i]);
    }
    if(maxSpeed > 1) {
      for (int i = 0; i < 4; i++){
        speeds[i] /= maxSpeed;
      }
    }
    fl.set(speeds[0]);
    fr.set(speeds[1]);
    bl.set(speeds[2]);
    br.set(speeds[3]);
  }

  private void enableAutoBulkRead() {
    for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
      mod.setBulkCachingMode(BulkCachingMode.AUTO);
    }
  }
}
