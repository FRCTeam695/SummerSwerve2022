package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ThreadClimb extends Thread
{
    
    public void run()
    {

        System.out.println("BEGIN ThreadClimb");

        while(DriverStation.isEnabled() && DriverStation.isTeleop())
        {

            Timer.delay(0.1);

            if (Robot.xbox.getPOV() == 0)
            {
                Robot.ClimbSolenoid.set(true);
            }

            if (Robot.xbox.getPOV() == 180)
            {
                Robot.ClimbSolenoid.set(false);
            }

            /*
            if (Robot.lstick.getRawButton(7) == true)
            {
                Robot.ClimbSolenoid.set(true);
            }
            if (Robot.lstick.getRawButton(8) == true)
            {
                Robot.ClimbSolenoid.set(false);
            }
            */
    
        }

        System.out.println("END ThreadClimb");

    }
    
}
