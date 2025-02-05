package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorHoldPositionCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;

    public ElevatorHoldPositionCommand(Elevator e){
    Elevator = e;
    ElevatorLeft = Elevator.ElevatorLeft;
    ElevatorRight = Elevator.ElevatorRight;
    Manipulator = Elevator.Manipulator;
    addRequirements(Elevator);
      
   }

    @Override
    public void execute(){

      ElevatorLeft.set(0);
      ElevatorRight.set(0);      

    }
    
    @Override
    public boolean isFinished() {
       return false;
    }

    }
