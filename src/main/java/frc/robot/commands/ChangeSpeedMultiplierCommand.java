package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveAugments;

public class ChangeSpeedMultiplierCommand extends Command {

    DriveAugments Augments;
    double wheels;
    double turn;

    public ChangeSpeedMultiplierCommand(DriveAugments d,double w, double t){
        Augments = d;
        wheels = w;
        turn = t;

        addRequirements(d);
   }

    @Override
    public void execute(){
        RobotContainer.SpeedMultiplier = wheels;
        RobotContainer.RotationSpeedMultiplier = turn;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public boolean isFinished() {
       return false;
    }

}
