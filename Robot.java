// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.kauailabs.navx.frc.AHRS;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static boolean shooting = false;

//  public static Joystick lstick = new Joystick(0);
//  public static Joystick rstick = new Joystick(1);
  public static Joystick xbox = new Joystick(4);

  // public static Compressor comp1 = new Compressor(0, PneumaticsModuleType.CTREPCM); no longer needed?

  public static Solenoid IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public static Solenoid ClimbSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  public static TalonFX Intake = new TalonFX(51);
  public static TalonFX Index = new TalonFX(52);

  public static TalonFX ShooterLow = new TalonFX(53);
  public static TalonFX ShooterHigh = new TalonFX(54);

  public static DigitalInput IndexBeam0 = new DigitalInput(0);
  public static DigitalInput IndexBeam1 = new DigitalInput(1);

  SendableChooser<Double> m_angleChooser = new SendableChooser<>();

  /*

  swerve drive chassis corner numbering:
      1x = front right
      2x = front left
      3x = rear left
      4x = rear right

      X=1:  cancoder
      X=2:  steering motor
      X=3:  drive motor

  */

  // cancoders
  public static WPI_CANCoder cancoder[] =
  {
    new WPI_CANCoder(11),
    new WPI_CANCoder(21),
    new WPI_CANCoder(31),
    new WPI_CANCoder(41)
  };

  // cancoder mounting orientation offsets (degrees)
  static double[] cancoderoffset = { 170, 228, 170, 204 };

  // steering motors
  public static TalonFX steer[] = 
  {
    new TalonFX(12),
    new TalonFX(22),
    new TalonFX(32),
    new TalonFX(42)
  };

  // drive motors
  public static TalonFX drive[] = 
  {
    new TalonFX(13),
    new TalonFX(23),
    new TalonFX(33),
    new TalonFX(43)
  };

  // default drive rotation directions (corner 1 is negated due to the camcoder mounting orientation)
  static double[] defaultrotation = { -1, 1, 1, 1 };

  // cancoder pid controllers
  public static PIDController cancoderpid[] =
  {
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001)
  };

  // talon encoder pid controllers (TODO:  move these directly into the falcon 500's)
  public static PIDController talonpid[] =
  {
    new PIDController(0.01, 0, 0),
    new PIDController(0.01, 0, 0),
    new PIDController(0.01, 0, 0),
    new PIDController(0.01, 0, 0)
  };

  // falcon encoder count per 360 degrees of steering rotation
  // falcon 500 is 2048 counts per 360 degrees of revolution
  // mk4i steering ratio is 150/7:1
  public static double talon_mk4i_360_count = 2048 * 150 / 7;

  // navx2 gyro
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  public static double nearzero(double val)
  {
    if (val > -0.01 && val < 0.01) val = 0.0;
    return(val);
  }

  // CancoderHome() takes 1 second to home the swerve steer wheels using the cancoders
  public static void CancoderHome()
  {
    System.out.println("Begin CancoderHome()");
    for(int t=0; t<100; t++)
    {
      for(int lp=0; lp<4; lp++)
      {
        double co = MathUtil.clamp(cancoderpid[lp].calculate(cancoder[lp].getAbsolutePosition(), cancoderoffset[lp]), -1, 1);
        steer[lp].set(ControlMode.PercentOutput, -1 * co);
      }
      Timer.delay(0.01);
    }
    for(int lp=0; lp<4; lp++)
    {
      steer[lp].setSelectedSensorPosition(0);
      talonpid[lp].reset();
    }
    System.out.println("End CancoderHome()");
  }

  public static void ShootHighFromHub()
  {
    double d;

    // spinup shooter wheels
    d = -0.60;
    //d = -0.30;
    Robot.ShooterLow.set(ControlMode.Velocity, d * 6100.0 * 2048.0 / 600.0);

    d = 0.55;
    //d = 0.35;
    Robot.ShooterHigh.set(ControlMode.Velocity, d * 6400.0 * 2048.0 / 600.0);

    // wait for spinup
    Timer.delay(0.75);

    // turn on indexer to shoot first cargo
    Robot.Index.set(ControlMode.PercentOutput, -0.3);
    while (Robot.IndexBeam0.get() == true)
    {
        Timer.delay(0.05);
    }

    // stop indexer after first cargo shot
    Robot.Index.set(ControlMode.PercentOutput, 0);

    // wait again for spinup
    Timer.delay(0.5);

    // turn on indexer to shoot second cargo
    Robot.Index.set(ControlMode.PercentOutput, -0.3);

    // wait for shot
    Timer.delay(1.0);

    // turn off shooter wheels and indexer
    Robot.ShooterLow.set(ControlMode.PercentOutput, 0);
    Robot.ShooterHigh.set(ControlMode.PercentOutput, 0);
    Robot.Index.set(ControlMode.PercentOutput, 0);

  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_angleChooser.setDefaultOption("No tarmac offset", 0.0);
    m_angleChooser.addOption("Left tarmac offset", 21.0);
    m_angleChooser.addOption("Right tarmac offset", -69.0);
    SmartDashboard.putData(m_angleChooser);

    gyro.calibrate();
    gyro.reset();

    Intake.configFactoryDefault();
    Index.configFactoryDefault();
    ShooterLow.configFactoryDefault();
    ShooterHigh.configFactoryDefault();

    Intake.setNeutralMode(NeutralMode.Coast);
    Index.setNeutralMode(NeutralMode.Brake);
    ShooterLow.setNeutralMode(NeutralMode.Coast);
    ShooterHigh.setNeutralMode(NeutralMode.Coast);

    // current limit drive falcons
    SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    falconlimit.enable = true;
    falconlimit.currentLimit = 20;
    falconlimit.triggerThresholdCurrent = 20;
    falconlimit.triggerThresholdTime = 0;

    for(int lp=0; lp<4; lp++)
    {
      steer[lp].configFactoryDefault();
      steer[lp].setNeutralMode(NeutralMode.Brake);
      steer[lp].configSupplyCurrentLimit(falconlimit);

      drive[lp].configFactoryDefault();
      drive[lp].setNeutralMode(NeutralMode.Brake);
      drive[lp].configSupplyCurrentLimit(falconlimit);

      // per ctre velocity control example
      drive[lp].configNeutralDeadband(0.001);
      drive[lp].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      drive[lp].configNominalOutputForward(0, 30);
		  drive[lp].configNominalOutputReverse(0, 30);
		  drive[lp].configPeakOutputForward(1, 30);
		  drive[lp].configPeakOutputReverse(-1, 30);

		  drive[lp].config_kF(0, 1023.0 / 20660.0, 30);
		  drive[lp].config_kP(0, 0.1, 30);
		  drive[lp].config_kI(0, 0.001, 30);
		  drive[lp].config_kD(0, 5, 30);
    }

    // closed loop velocity control on low shooter
    ShooterLow.configNeutralDeadband(0.001);
    ShooterLow.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    ShooterLow.configNominalOutputForward(0, 30);
    ShooterLow.configNominalOutputReverse(0, 30);
    ShooterLow.configPeakOutputForward(1, 30);
    ShooterLow.configPeakOutputReverse(-1, 30);

    ShooterLow.config_kF(0, 1023.0 / 20660.0, 30);
    ShooterLow.config_kP(0, 0.1, 30);
    ShooterLow.config_kI(0, 0.001, 30);
    ShooterLow.config_kD(0, 5, 30);

    // closed loop velocity control on high shooter
    ShooterHigh.configNeutralDeadband(0.001);
    ShooterHigh.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    ShooterHigh.configNominalOutputForward(0, 30);
    ShooterHigh.configNominalOutputReverse(0, 30);
    ShooterHigh.configPeakOutputForward(1, 30);
    ShooterHigh.configPeakOutputReverse(-1, 30);

    ShooterHigh.config_kF(0, 1023.0 / 20660.0, 30);
    ShooterHigh.config_kP(0, 0.1, 30);
    ShooterHigh.config_kI(0, 0.001, 30);
    ShooterHigh.config_kD(0, 5, 30);

    // kickoff display thread to show stuff on shuffleboard
    ThreadDisplay th_display = new ThreadDisplay();
    th_display.start();
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    CancoderHome();
    ShootHighFromHub();

    //Backup 6 feet
    Robot.drive[0].setSelectedSensorPosition(0, 0, 100);
    Robot.drive[0].set(ControlMode.PercentOutput, .30);
    Robot.drive[1].set(ControlMode.PercentOutput, -.30);
    Robot.drive[2].set(ControlMode.PercentOutput, -.30);
    Robot.drive[3].set(ControlMode.PercentOutput, -.30);
    while(Robot.drive[0].getSelectedSensorPosition(0) < 115000);
    {
      Timer.delay(0.01);
    }
    Robot.drive[0].set(ControlMode.PercentOutput, 0);
    Robot.drive[1].set(ControlMode.PercentOutput, 0);
    Robot.drive[2].set(ControlMode.PercentOutput, 0);
    Robot.drive[3].set(ControlMode.PercentOutput, 0);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    Robot.shooting = false;

    Robot.drive[0].setSelectedSensorPosition(0, 0, 100);

    ThreadIntake th_intake = new ThreadIntake();
    th_intake.start();
    
    ThreadClimb th_climb = new ThreadClimb();
    th_climb.start();
    
  }

/** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // min and max steering motor percent output
    double MinSteer = -1.0;
    double MaxSteer = 1.0;

    // temporary variables for setpoint and control output
    double sp;
    double co;

    // chassis dimensions
    double L = 25;
    double W = 25;
    double R = Math.sqrt(L*L + W*W);

    // read joystick values
//    double Xj = Math.pow(lstick.getRawAxis(0), 3);
//    double Yj = Math.pow(lstick.getRawAxis(1), 3);
//    double Zj = rstick.getRawAxis(0);

    double Xj = Math.pow(xbox.getRawAxis(0), 3);
    double Yj = Math.pow(xbox.getRawAxis(1), 3);
    double Zj = xbox.getRawAxis(4);

    // if xbox A button is pressed, home the drive wheels using the cancoders, reset the falcon encoders, and exit
    //  if (rstick.getRawButton(7) == true)
    if (xbox.getRawButton(1) == true)
    {
      CancoderHome();
      return;
    }

    // if xbox B button is pressed, reset gyro
    if (Robot.xbox.getRawButton(2) == true)
    {
      Robot.gyro.reset();
      return;
    }

    SmartDashboard.putNumber("Yaw", Robot.gyro.getYaw());
    SmartDashboard.putBoolean("Beam", Robot.IndexBeam0.get());
    SmartDashboard.putBoolean("Shooting", Robot.shooting);

    // convert joystick values to strafe, forward, and rotate
    double deadband = 0.075;
    double STR = Xj;
    if (STR > -deadband && STR < deadband) STR = 0;

    double FWD = -Yj;
    if (FWD > -deadband && FWD < deadband) FWD = 0;

    double RCW = Zj;
    if (RCW > -deadband && RCW < deadband) RCW = 0;

    // limit rotate to 20% motor
    RCW /= 3;

    // adjust for field oriented drive
    double gyro_rad = (gyro.getYaw() + m_angleChooser.getSelected()) / 180 * Math.PI;
    //double gyro_rad = gyro.getYaw() / 180 * Math.PI;
    double tFWD = FWD * Math.cos(gyro_rad) + STR * Math.sin(gyro_rad);
    STR = -FWD * Math.sin(gyro_rad) + STR * Math.cos(gyro_rad);
    FWD = tFWD;

    SmartDashboard.putNumber("STR", STR);
    SmartDashboard.putNumber("FWD", FWD);
    SmartDashboard.putNumber("RCW", RCW);

    // compute temporary work variables
    double A = nearzero(STR + RCW * (L/R));
    double B = nearzero(STR - RCW * (L/R));
    double C = nearzero(FWD + RCW * (W/R));
    double D = nearzero(FWD - RCW * (W/R));

    // compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
    };

    // compute speeds
    double[] speed =
    {
      Math.sqrt(A*A + D*D),
      Math.sqrt(A*A + C*C),
      Math.sqrt(B*B + C*C),
      Math.sqrt(B*B + D*D)
    };

    // set default drive rotations
    double[] rotation =
    {
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
    };

    // normalize speeds for motor controllers (range is 0 to 1)
    double max = 0;
    for(int lp=0; lp<4; lp++) if (speed[lp] > max) max = speed[lp];
    if (max > 1) for(int lp=0; lp<4; lp++) speed[lp] /= max;
    
    // only run motors if nonzero speed(s)
    if (max != 0)
    {

      // loop to compute and apply steering angle and drive power to each motor
      for(int lp=0; lp<4; lp++)
      {

        // get target steering angle (setpoint) for this corner
        // (invert because falcons are upside down)
        sp = angle[lp];

        // get current steering angle (process variable) for this corner
        double ec = steer[lp].getSelectedSensorPosition(0);
        double pv = (ec % talon_mk4i_360_count) / talon_mk4i_360_count * 360;
        if (pv <= -180)
        {
          pv += 360;
        }
        else if (pv >= 180)
        {
          pv -= 360;
        }

        // compute steering angle change
        double delta = Math.abs(sp - pv);

        // adjust steering angle for optimized turn
        if (delta > 90 && delta < 270)
        {
          rotation[lp] = -defaultrotation[lp];
          sp = (sp >= 0) ? sp - 180 : sp + 180;
        }

        // adjust angle orientation for PID depending on direction of setpoint
        if (sp < -135 || sp > 135)
        {
          if (sp < 0)
          {
            sp += 360;
          }
          if (pv < 0)
          {
            pv += 360;
          }
        }

        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);

        // send drive output to falcon
        //drive[lp].set(ControlMode.Velocity, speed[lp]);
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

        // convenient place to do cancoder home control loop cleanup
        cancoderpid[lp].reset();
      }
    }

    // otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        steer[lp].set(ControlMode.PercentOutput, 0);
        drive[lp].set(ControlMode.PercentOutput, 0);
        //drive[lp].set(ControlMode.Velocity, 0);
      }
    }
 
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() { }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic(){
    if (Robot.xbox.getRawButton(2) == true)
    {
      Robot.gyro.reset();
      return;
    }
    SmartDashboard.putNumber("Yaw", Robot.gyro.getYaw());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}