package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeSpeedMultiplierCommand;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveAugments implements Subsystem {
    public CommandXboxController Driver;
    ChangeSpeedMultiplierCommand slowMode;
    ChangeSpeedMultiplierCommand resetSpeed;
    ChangeSpeedMultiplierCommand ultraSlowMode;

    public DriveAugments(CommandXboxController bruh){
        Driver = bruh;
        Trigger leftTrigger = Driver.leftTrigger();
        Trigger rightTrigger = Driver.rightTrigger();
        slowMode = new ChangeSpeedMultiplierCommand(this,0.35,0.5);
        ultraSlowMode = new ChangeSpeedMultiplierCommand(this,0.15,0.25);


        leftTrigger.whileTrue(slowMode);
        rightTrigger.whileTrue(ultraSlowMode);


        resetSpeed = new ChangeSpeedMultiplierCommand(this,1,1);

        this.setDefaultCommand(resetSpeed);
    }   

    @Override
    public void periodic(){
    
    }
}



