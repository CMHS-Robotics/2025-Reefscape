package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorHoldPositionCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   MotionMagicVoltage positionControl;
   Angle motPos;

   public ElevatorHoldPositionCommand(Elevator e){
      Elevator = e;
      ElevatorLeft = Elevator.ElevatorLeft;
      ElevatorRight = Elevator.ElevatorRight;
      Manipulator = Elevator.Manipulator;
      positionControl = new MotionMagicVoltage(0).withSlot(0);
      addRequirements(Elevator);
   }

   @Override
   public void initialize(){
      motPos = Elevator.motorPosition;
   }


    @Override
    public void execute(){

      // ElevatorLeft.set(0);
      // ElevatorRight.set(0);
      SmartDashboard.putNumber("positionControl",motPos.magnitude());
      SmartDashboard.putString("Command Running:","HoldPosition");
      SmartDashboard.putString("bruh",ElevatorLeft.toString());
      
      ElevatorLeft.setControl(positionControl.withPosition(motPos).withSlot(0));
      ElevatorRight.setControl(positionControl.withPosition(motPos).withSlot(0));

    }
    
    @Override
    public boolean isFinished() {
       return false;
    }

    public Angle getMotPos() {
        return motPos;
    }

    }
