package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorToStageCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;

    public ElevatorToStageCommand(Elevator e){
    Elevator = e;
    ElevatorLeft = e.ElevatorLeft;
    ElevatorRight = e.ElevatorRight;
    Manipulator = e.Manipulator;
    addRequirements(e);
      
   }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }
    
    @Override
    public boolean isFinished() {
       return false;
    }
    }