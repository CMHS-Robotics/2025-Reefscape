package frc.robot.tools;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.CoralSetSpinSpeedCommandV2;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    CommandSwerveDrivetrain Swerve;

    //entries
    GenericEntry leftMotorPos;
    GenericEntry rightMotorPos;
    GenericEntry pidTarget;
    GenericEntry elevatorLevel;
    GenericEntry elevatorPID;
    GenericEntry coralWristPID;
    GenericEntry coralWristPosition;
    GenericEntry coralSpin;
    GenericEntry coralWristSetPosManual;
    GenericEntry coralWristManual;

    //shuffleboard tabs
    ShuffleboardTab DataTab = Shuffleboard.getTab("Data");
    ShuffleboardTab CommandsTab = Shuffleboard.getTab("Commands");

    //layouts
    ShuffleboardLayout ElevatorCommands = CommandsTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,10).withPosition(0,0);
    ShuffleboardLayout CoralCommands = CommandsTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,10).withPosition(2,0);
    ShuffleboardLayout Subsystems = CommandsTab.getLayout("Subsystems",BuiltInLayouts.kList).withSize(2,10).withPosition(4,0);
    ShuffleboardLayout ElevatorData = DataTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,10).withPosition(0,0);
    ShuffleboardLayout CoralData = DataTab.getLayout("Coral",BuiltInLayouts.kList).withSize(2,10).withPosition(2,0);

    public ShuffleboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w){
        Elevator = e;
        CoralSpin = s;
        CoralWrist = w;     
        
        DataTab.add("Motor",Elevator.ElevatorLeft);

        DataTab.add("Pigeon",new Pigeon2(0));

        //elevator data
        leftMotorPos = ElevatorData.add("Elevator Left Motor Position",0).getEntry();
        rightMotorPos = ElevatorData.add("Elevator Right Motor Position",0).getEntry();
        pidTarget = ElevatorData.add("Elevator PID Target",0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Bottom",0,"Top",24)).withSize(2,1).getEntry();
        elevatorLevel = ElevatorData.add("Elevator Level",0).getEntry();
        elevatorPID = ElevatorData.add("Elevator PID",0).getEntry();

        //coral data
        coralWristPosition = CoralData.add("Coral Wrist Position",0).getEntry();
        coralWristPID = CoralData.add("Coral PID",0).getEntry();
        coralSpin = CoralData.add("Coral Spin Value",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        coralWristManual = CoralData.add("Manual PID setting",false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        coralWristSetPosManual = CoralData.add("PID Bar",0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Bottom",-500,"Top",500)).getEntry();


         //subsystems
        //  SmartDashboard.putData((Sendable) Elevator);
         
        Subsystems.add("Elevator",Elevator);
        Subsystems.add("Coral Spin",CoralSpin);
        Subsystems.add("Coral Wrist",CoralWrist);
        Subsystems.add("Driver Augments",DriveAugments);
        Subsystems.add("Swerve",Swerve);
 
         //commands
        //  SmartDashboard.putData("Elevator Bottom",new ElevatorSetStageCommand(Elevator,0));
        //  SmartDashboard.putData("Elevator Top",new ElevatorSetStageCommand(Elevator,4));

        ElevatorCommands.add("Elevator Top",new ElevatorSetStageCommand(Elevator,4));
        ElevatorCommands.add("Elevator L3",new ElevatorSetStageCommand(Elevator,3));
        ElevatorCommands.add("Elevator L2",new ElevatorSetStageCommand(Elevator,2));
        ElevatorCommands.add("Elevator Intake",new ElevatorSetStageCommand(Elevator,1));
        ElevatorCommands.add("Elevator Bottom",new ElevatorSetStageCommand(Elevator,0));

        CoralCommands.add("Run Coral at x", new CoralSetSpinSpeedCommandV2(CoralSpin,coralSpin.getDouble(0)));



         this.setDefaultCommand(Commands.run(()-> update(),this));
    }





    public void update(){

        //elevator
        leftMotorPos.setDouble(Elevator.ElevatorLeft.getPosition().getValueAsDouble());
        rightMotorPos.setDouble(Elevator.ElevatorRight.getPosition().getValueAsDouble());

        pidTarget.setDouble(Elevator.getTargetPosition());
        Elevator.setTargetPosition(pidTarget.getDouble(0));

        elevatorLevel.setInteger(Elevator.getStageLevel());
        elevatorPID.setString(Elevator.elevatorPID.toString());
        
        CoralWrist.setShuffleboardManualControl(coralWristManual.getBoolean(false));
        CoralWrist.setShuffleboardManualControlValue(coralWristSetPosManual.getDouble(0.0));

        //coral
        coralWristPID.setString(CoralWrist.coralWristPID.toString());
        coralWristPosition.setDouble(CoralWrist.CoralWrist.getPosition().getValueAsDouble());





    }

}
