package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ThreadIntake extends Thread
{

    public void run()
    {
        System.out.println("BEGIN ThreadIntake");

        while(DriverStation.isEnabled() && DriverStation.isTeleop())
        {

            Timer.delay(0.050);

            double d;

            // check if intake deploy requested
//            if (Robot.lstick.getRawButton(1) == true)
            SmartDashboard.putNumber("RawAxis3", Robot.xbox.getRawAxis(3));
            if (Robot.xbox.getRawAxis(3) > 0.2)
            {

                // deploy intake pistons
                Robot.IntakeSolenoid.set(true);

                // run intake motor
                Robot.Intake.set(ControlMode.PercentOutput, 0.35);

                // run indexer if no captive cargo
                if (Robot.IndexBeam0.get() == false)
                {
                    Robot.Index.set(ControlMode.PercentOutput, -0.20);
                }

                // otherwise, do not run indexer
                else
                {
                    Robot.Index.set(ControlMode.PercentOutput, 0);
                }

            }
            // otherwise, retract intake
            else
            {

                // retract intake pistons and stop motors
                Robot.IntakeSolenoid.set(false);
                Robot.Intake.set(ControlMode.PercentOutput, 0);
                Robot.Index.set(ControlMode.PercentOutput, 0);
            }

            // discharge cargo through intake
            if(Robot.xbox.getRawButton(3) == true)
            {
                //Deploy intake
                Robot.IntakeSolenoid.set(true);

                //Run out intake motor
                Robot.Intake.set(ControlMode.PercentOutput, -0.35);

                //Run out index motor
                Robot.Index.set(ControlMode.PercentOutput, 0.20);

            }

            // discharge cargo through shooter
            if(Robot.xbox.getRawButton(4) == true)
            {
                //Run index motor towards shooter wheel
                Robot.Index.set(ControlMode.PercentOutput, -0.20);

                //Set shooter motors at "reject" percentages
                Robot.ShooterLow.set(ControlMode.PercentOutput, -0.18);
                Robot.ShooterHigh.set(ControlMode.PercentOutput, 0.15);

                Timer.delay(3);

                Robot.ShooterLow.set(ControlMode.PercentOutput, 0);
                Robot.ShooterHigh.set(ControlMode.PercentOutput, 0);                

            }

            // check if shooter requested (assumes robot front bumper is against hub fender)
//            if (Robot.rstick.getRawButton(1) == true)
            if (Robot.xbox.getRawAxis(2) > 0.2)
            {
                Robot.shooting = true;
                Robot.ShootHighFromHub();
                Robot.shooting = false;
            }

        }

        System.out.println("END ThreadIntake");

    }

}
