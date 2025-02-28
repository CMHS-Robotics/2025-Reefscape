package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
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
    Elevator elevator;

    public DriveAugments(CommandXboxController bruh,Elevator e){
        Driver = bruh;
        elevator = e;
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
        SmartDashboard.putNumber("Speed Multiplier",RobotContainer.ElevatorMultiplier * RobotContainer.SpeedMultiplier);
        switch(elevator.getStageLevel()){
            case 0 -> {RobotContainer.ElevatorMultiplier = 1;
                RobotContainer.ElevatorRotationMultiplier = 1;}
            case 1 -> {RobotContainer.ElevatorMultiplier = 0.7;
                RobotContainer.ElevatorRotationMultiplier = 0.75;}
            case 2 -> {RobotContainer.ElevatorMultiplier = 0.45;
                RobotContainer.ElevatorRotationMultiplier = 0.6;}
            case 3 -> {RobotContainer.ElevatorMultiplier = 0.25;
                RobotContainer.ElevatorRotationMultiplier = 0.35;}
            case 4 -> {RobotContainer.ElevatorMultiplier = 0.15;
                RobotContainer.ElevatorRotationMultiplier = 0.25;}
            default -> {RobotContainer.ElevatorMultiplier = 1;
                RobotContainer.ElevatorRotationMultiplier = 1;}
        }
    }
}



