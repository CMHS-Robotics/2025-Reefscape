package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorToStageCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   Angle position;

    public ElevatorToStageCommand(Elevator e, Angle a){
    Elevator = e;
    ElevatorLeft = e.ElevatorLeft;
    ElevatorRight = e.ElevatorRight;
    Manipulator = e.Manipulator;
    position = a;
    addRequirements(e);
      
   }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
      Manipulator.setRumble(RumbleType.kBothRumble, 1);
      ElevatorLeft.setPosition(position);

    }
    
    @Override
    public boolean isFinished() {
       return false;
    }
    }