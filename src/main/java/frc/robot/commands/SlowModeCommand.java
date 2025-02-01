package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveAugments;

public class SlowModeCommand extends Command {

    public SlowModeCommand(DriveAugments d){
        addRequirements(d);
      
   }

    @Override
    public void execute(){
        RobotContainer.SpeedMultiplier = 0.5;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public boolean isFinished() {
       return false;
    }

}
