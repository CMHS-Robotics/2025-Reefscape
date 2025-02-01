package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorToStageCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   Angle position;
   final PositionVoltage bruh;
   MotionMagicVoltage pos;

    public ElevatorToStageCommand(Elevator e, Angle a, MotionMagicVoltage p){
    Elevator = e;
    ElevatorLeft = Elevator.ElevatorLeft;
    ElevatorRight = Elevator.ElevatorRight;
    Manipulator = Elevator.Manipulator;
    position = a;
    pos = p;
    bruh = new PositionVoltage(position).withSlot(0);

    addRequirements(Elevator);
      
   }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

      ElevatorLeft.setControl(bruh);

      //ElevatorLeft.setControl(pos.withPosition(position));
      
    }
    
    public void end(){
    }

    @Override
    public boolean isFinished() {
       return false;
    }

    public boolean targetReached(){
      boolean nuts = (Math.abs(ElevatorLeft.getPosition().getValueAsDouble() - position.baseUnitMagnitude()) < 0.1);
      
      
      return nuts;
    }
   }