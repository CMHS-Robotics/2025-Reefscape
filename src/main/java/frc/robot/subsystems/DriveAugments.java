package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChangeSpeedMultiplierCommand;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveAugments extends SubsystemBase {
    public CommandXboxController Driver;
    ChangeSpeedMultiplierCommand speedMod;
    Elevator elevator;
    int elevatorStage;
    double[][] speedsAtLevels = {//array 1, elevator stage //array 2, (basetranslationspeed,baserotationspeed,slowtranslationspeed,slowrotationspeed,superslowtranslationspeed,superslowrotationspeed)
    {1,1,0.35,0.5,0.15,0.25},
    {0.7,0.75,0.3,0.40,0.13,0.22},
    {0.45,0.6,0.23,0.35,0.11,0.20},
    {0.25,0.35,0.15,0.25,0.09,0.18},
    {0.15,0.25,0.1,0.2,0.08,0.15}
    };

    public DriveAugments(CommandXboxController x,Elevator e){
        Driver = x;
        elevator = e;
        elevatorStage = elevator.getStageLevel();

        Trigger leftTrigger = Driver.leftTrigger();
        Trigger rightTrigger = Driver.rightTrigger();

        speedMod = new ChangeSpeedMultiplierCommand(this);

        //run the speedMod command with the paremeters based on the 2d array so it updates based on the stage of the elevator
        leftTrigger.whileTrue(speedMod.run(speedsAtLevels[elevatorStage][2],speedsAtLevels[elevatorStage][3]));
        rightTrigger.whileTrue(speedMod.run(speedsAtLevels[elevatorStage][4],speedsAtLevels[elevatorStage][5]));

        setDefaultCommand(speedMod.run(speedsAtLevels[elevatorStage][0],speedsAtLevels[elevatorStage][1]));
    }   

    @Override
    public void periodic(){
        elevatorStage = elevator.getStageLevel();
    
    }
}



