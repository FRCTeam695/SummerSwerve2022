package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThreadDisplay extends Thread
{
    
    public void run()
    {

        System.out.println("BEGIN ThreadDisplay");

        while(true)
        {
            Timer.delay(0.100);
        }

    }
    
}
