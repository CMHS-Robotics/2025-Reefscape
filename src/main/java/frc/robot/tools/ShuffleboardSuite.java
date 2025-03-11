package frc.robot.tools;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.subsystems.CoralSpinV2;
import frc.robot.subsystems.CoralWristV2;
import frc.robot.subsystems.DriveAugments;
import frc.robot.subsystems.Elevator;

public class ShuffleboardSuite implements Subsystem{
    
    //subsystems
    Elevator Elevator;
    CoralSpinV2 CoralSpin;
    CoralWristV2 CoralWrist;
    DriveAugments DriveAugments;

    //entries
    GenericEntry leftMotorPos;
    GenericEntry rightMotorPos;
    GenericEntry pidTarget;
    GenericEntry elevatorLevel;
    GenericEntry elevatorPID;

    //shuffleboard tabs
    ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");
    ShuffleboardTab CoralTab = Shuffleboard.getTab("Coral Intake");
    ShuffleboardTab CommandsTab = Shuffleboard.getTab("Commands");

    //layouts
    ShuffleboardLayout ElevatorCommands = CommandsTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,2).withPosition(0,0);
    ShuffleboardLayout Subsystems = CommandsTab.getLayout("Subsystems",BuiltInLayouts.kList).withSize(2,2).withPosition(2,0);
    ShuffleboardLayout ElevatorData = ElevatorTab.getLayout("Data",BuiltInLayouts.kList).withSize(2,4).withPosition(0,0);


    public ShuffleboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w){
        Elevator = e;
        CoralSpin = s;
        CoralWrist = w;     
        
        Shuffleboard.getTab("Elevator").add("Motor",Elevator.ElevatorLeft);

        leftMotorPos = ElevatorData.add("Elevator Left Motor Position",0).getEntry();
        rightMotorPos = ElevatorData.add("Elevator Right Motor Position",0).getEntry();
        pidTarget = ElevatorData.add("Elevator PID Target",0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Bottom",0,"Top",24)).withSize(2,1).getEntry();
        elevatorLevel = ElevatorData.add("Elevator Level",0).getEntry();
        elevatorPID = ElevatorData.add("Elevator PID",0).getEntry();

         //subsystems
        //  SmartDashboard.putData((Sendable) Elevator);
         
        Subsystems.add("Elevator",Elevator);
        Subsystems.add("Coral Spin",CoralSpin);
        Subsystems.add("Coral Wrist",CoralWrist);
        Subsystems.add("Driver Augments",DriveAugments);
 
         //commands
        //  SmartDashboard.putData("Elevator Bottom",new ElevatorSetStageCommand(Elevator,0));
        //  SmartDashboard.putData("Elevator Top",new ElevatorSetStageCommand(Elevator,4));

        ElevatorCommands.add("Elevator Top",new ElevatorSetStageCommand(Elevator,4));
        ElevatorCommands.add("Elevator L3",new ElevatorSetStageCommand(Elevator,3));
        ElevatorCommands.add("Elevator L2",new ElevatorSetStageCommand(Elevator,2));
        ElevatorCommands.add("Elevator Intake",new ElevatorSetStageCommand(Elevator,1));
        ElevatorCommands.add("Elevator Bottom",new ElevatorSetStageCommand(Elevator,0));







         this.setDefaultCommand(Commands.run(()->update()));
    }





    public void update(){

        
        leftMotorPos.setDouble(Elevator.ElevatorLeft.getPosition().getValueAsDouble());
        rightMotorPos.setDouble(Elevator.ElevatorRight.getPosition().getValueAsDouble());

        pidTarget.setDouble(Elevator.getTargetPosition());
        Elevator.setTargetPosition(pidTarget.getDouble(0));
        

        elevatorLevel.setInteger(Elevator.getStageLevel());
        elevatorPID.setString(Elevator.elevatorPID.toString());
        

    }

}
