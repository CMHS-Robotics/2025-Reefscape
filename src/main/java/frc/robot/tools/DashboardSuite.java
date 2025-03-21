package frc.robot.tools;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CoralWristSetTargetPositionCommand;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.commands.ZeroTalonCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSpinV2;
import frc.robot.subsystems.CoralWristV2;
import frc.robot.subsystems.DriveAugments;
import frc.robot.subsystems.Elevator;

public class DashboardSuite extends SubsystemBase{
    
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
    GenericEntry elevatorSetPosManual;
    GenericEntry elevatorManual;
    GenericEntry coralWristOutput;

    //shuffleboard tabs
    ShuffleboardTab DataTab = Shuffleboard.getTab("Data");
    ShuffleboardTab CommandsTab = Shuffleboard.getTab("Commands");

    
    //layouts
    ShuffleboardLayout ElevatorCommands = CommandsTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,10).withPosition(0,0);
    ShuffleboardLayout CoralCommands = CommandsTab.getLayout("Coral",BuiltInLayouts.kList).withSize(2,10).withPosition(2,0);
    ShuffleboardLayout ElevatorData = DataTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(3,10).withPosition(0,0);
    ShuffleboardLayout CoralData = DataTab.getLayout("Coral",BuiltInLayouts.kList).withSize(3,10).withPosition(3,0);

    public DashboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w){
        Elevator = e;
        CoralSpin = s;
        CoralWrist = w;     
        //elevator data
        // leftMotorPos = ElevatorData.add("Elevator Left Motor Position",0).getEntry();
        // rightMotorPos = ElevatorData.add("Elevator Right Motor Position",0).getEntry();
        // pidTarget = ElevatorData.add("Elevator PID Target",0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min_value",0,"max_value",24)).withSize(2,1).getEntry();
        // elevatorLevel = ElevatorData.add("Elevator Level",0).getEntry();
        // elevatorPID = ElevatorData.add("Elevator PID","").getEntry();
        // elevatorManual = ElevatorData.add("Manual PID Setting",false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        // elevatorSetPosManual = ElevatorData.add("PID Target Value Manual",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

        // //coral data
        // coralWristPosition = CoralData.add("Coral Wrist Position",0).getEntry();
        // coralWristOutput = CoralData.add("Coral Wrist Output",0).getEntry();
        // coralWristPID = CoralData.add("Coral PID","").getEntry();
        // coralSpin = CoralData.add("Coral Spin Value",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        // coralWristManual = CoralData.add("Manual PID setting",false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        // coralWristSetPosManual = CoralData.add("PID Target Value Manual",0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min_value",-3,"max_value",3 )).getEntry();


        //  //subsystems
        // // Subsystems.add("Elevator",Elevator);
        // // Subsystems.add("Coral Spin",CoralSpin);
        // // Subsystems.add("Coral Wrist",CoralWrist);
        // // Subsystems.add("Driver Augments",DriveAugments);
        // // Subsystems.add("Swerve",Swerve);
 
        //  //commands
        // ElevatorCommands.add("Elevator Top",new ElevatorSetStageCommand(Elevator,4).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 3)));
        // ElevatorCommands.add("Elevator L3",new ElevatorSetStageCommand(Elevator,3).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        // ElevatorCommands.add("Elevator L2",new ElevatorSetStageCommand(Elevator,2).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        // ElevatorCommands.add("Elevator Intake",new ElevatorSetStageCommand(Elevator,1).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 1)));
        // ElevatorCommands.add("Elevator Bottom",new ElevatorSetStageCommand(Elevator,0).alongWith(new CoralWristSetTargetPositionCommand(CoralWrist, 0)));

        SmartDashboard.putData("Elevator Top Command",new ElevatorSetStageCommand(Elevator,4).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 3)));
        SmartDashboard.putData("Elevator Bottom Command",new ElevatorSetStageCommand(Elevator,0).alongWith(new CoralWristSetTargetPositionCommand(CoralWrist, 0)));

        SmartDashboard.putData("Zero All Motors",new ZeroTalonCommand(Elevator.ElevatorLeft).alongWith(new ZeroTalonCommand(Elevator.ElevatorRight)).alongWith(new ZeroTalonCommand(CoralWrist.CoralWrist)));

        SmartDashboard.putData("Elevator",Elevator);
        SmartDashboard.putData("Wrist",CoralWrist);
        SmartDashboard.putData("Spin",CoralSpin);

        // CoralCommands.add("Run Coral at x", new CoralSetSpinSpeedCommandV2(CoralSpin,coralSpin.getDouble(0)));
    
    }




    @Override
    public void periodic(){

        // //elevator
        // leftMotorPos.setDouble(Elevator.ElevatorLeft.getPosition().getValueAsDouble());
        // rightMotorPos.setDouble(Elevator.ElevatorRight.getPosition().getValueAsDouble());

        // pidTarget.setDouble(Elevator.getTargetPosition());

        // elevatorLevel.setInteger(Elevator.getStageLevel());
        // elevatorPID.setString(Elevator.elevatorPID.toString());
        
        // if(elevatorManual.getBoolean(false)){
        //     Elevator.setTargetPosition(elevatorSetPosManual.getDouble(0.0));
        // }
        
        // //coral
        // coralWristPID.setString(CoralWrist.coralWristPID.toString());
        // coralWristPosition.setDouble(CoralWrist.CoralWrist.getPosition().getValueAsDouble());
        // coralWristOutput.setDouble(CoralWrist.CoralWrist.get());

        // CoralWrist.setShuffleboardManualControl(coralWristManual.getBoolean(false));
        // CoralWrist.setShuffleboardManualControlValue(coralWristSetPosManual.getDouble(0.0));

        // //Commands.print("" + elevatorSetPosManual.getDouble(0.0));



    }

}
