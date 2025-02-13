package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorFreeMoveCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;

    public ElevatorFreeMoveCommand(Elevator e){
    Elevator = e;
    ElevatorLeft = Elevator.ElevatorLeft;
    ElevatorRight = Elevator.ElevatorRight;
    Manipulator = Elevator.Manipulator;
    addRequirements(Elevator);
      
   }

    @Override
    public void execute(){
      SmartDashboard.putString("Command Running:","FreeMove");
      ElevatorLeft.set(-0.2 * Manipulator.getRightY());
      //ElevatorRight.set(-0.2 * Manipulator.getRightY());
      Elevator.motorPosition = ElevatorLeft.getPosition().getValue();
      Elevator.targetPosition = ElevatorLeft.getPosition().getValueAsDouble();
      ElevatorLeft.setNeutralMode(Brake);
      ElevatorRight.setNeutralMode(Brake);

    }
    @Override
    public boolean isFinished() {
       return false;
    }

    }
