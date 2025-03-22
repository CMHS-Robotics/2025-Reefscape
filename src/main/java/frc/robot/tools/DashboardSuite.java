package frc.robot.tools;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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

    // //entries
    // GenericEntry leftMotorPos;
    // GenericEntry rightMotorPos;
    // GenericEntry pidTarget;
    // GenericEntry elevatorLevel;
    // GenericEntry elevatorPID;
    // GenericEntry coralWristPID;
    // GenericEntry coralWristPosition;
    // GenericEntry coralSpin;
    // GenericEntry coralWristSetPosManual;
    // GenericEntry coralWristManual;
    // GenericEntry elevatorSetPosManual;
    // GenericEntry elevatorManual;
    // GenericEntry coralWristOutput;

    //subscribers
    //elevator
    DoubleSubscriber sElevatorPIDBar;
    BooleanSubscriber sElevatorPIDManual;
    DoubleSubscriber sElevatorPIDP;
    DoubleSubscriber sElevatorPIDD;
    DoubleSubscriber sElevatorPIDClampUpper;
    DoubleSubscriber sElevatorPIDClampLower;
 
    //publishers
    //elevator
    DoublePublisher pElevatorLeftMotor;
    DoublePublisher pElevatorRightMotor;
    StringPublisher pElevatorPID;
    StringPublisher pElevatorPIDSettings;
    BooleanPublisher pElevatorHasReached;
    DoublePublisher pElevatorPIDResult;

    // //shuffleboard tabs
    // ShuffleboardTab DataTab = Shuffleboard.getTab("Data");
    // ShuffleboardTab CommandsTab = Shuffleboard.getTab("Commands");

    
    //layouts
    // ShuffleboardLayout ElevatorCommands = CommandsTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(2,10).withPosition(0,0);
    // ShuffleboardLayout CoralCommands = CommandsTab.getLayout("Coral",BuiltInLayouts.kList).withSize(2,10).withPosition(2,0);
    // ShuffleboardLayout ElevatorData = DataTab.getLayout("Elevator",BuiltInLayouts.kList).withSize(3,10).withPosition(0,0);
    // ShuffleboardLayout CoralData = DataTab.getLayout("Coral",BuiltInLayouts.kList).withSize(3,10).withPosition(3,0);

    public DashboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w){
        Elevator = e;
        CoralSpin = s;
        CoralWrist = w;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        //elevator data
        NetworkTable ElevatorData = inst.getTable("Elevator");

        ElevatorData.getDoubleTopic("Elevator PID Bar Manual Target").publish();

        sElevatorPIDBar = ElevatorData.getDoubleTopic("Elevator PID Bar Manual Target").subscribe(0.0);

        ElevatorData.getBooleanTopic("Elevator Manual Control Active").publish();

        sElevatorPIDManual = ElevatorData.getBooleanTopic("Elevator Manual Control Active").subscribe(false);

        ElevatorData.getDoubleTopic("Elevator PID P Value").publish().set(0.2);
        ElevatorData.getDoubleTopic("Elevator PID D Value").publish().set(0.4);
        ElevatorData.getDoubleTopic("Elevator PID Lower Limit").publish().set(-0.15);
        ElevatorData.getDoubleTopic("Elevator PID Upper Limit").publish().set(0.4);
        pElevatorLeftMotor = ElevatorData.getDoubleTopic("Elevator Left Motor").publish();
        pElevatorRightMotor = ElevatorData.getDoubleTopic("Elevator Right Motor").publish();
        pElevatorPID = ElevatorData.getStringTopic("Elevator PID").publish();
        pElevatorPIDSettings = ElevatorData.getStringTopic("Elevator PID Settings").publish();
        pElevatorPIDResult = ElevatorData.getDoubleTopic("Elevator PID Result").publish();
        pElevatorHasReached = ElevatorData.getBooleanTopic("Elevator Has Reached Target").publish();



        sElevatorPIDP = ElevatorData.getDoubleTopic("Elevator PID P Value").subscribe(0);
        sElevatorPIDD = ElevatorData.getDoubleTopic("Elevator PID D Value").subscribe(0);
        sElevatorPIDClampLower = ElevatorData.getDoubleTopic("Elevator PID Lower Limit").subscribe(0);
        sElevatorPIDClampUpper = ElevatorData.getDoubleTopic("Elevator PID Upper Limit").subscribe(0);

        // //coral data
        // coralWristPosition = CoralData.add("Coral Wrist Position",0).getEntry();
        // coralWristOutput = CoralData.add("Coral Wrist Output",0).getEntry();
        // coralWristPID = CoralData.add("Coral PID","").getEntry();
        // coralSpin = CoralData.add("Coral Spin Value",0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        // coralWristManual = CoralData.add("Manual PID setting",false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        // coralWristSetPosManual = CoralData.add("PID Target Value Manual",0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min_value",-3,"max_value",3 )).getEntry();
 
        SmartDashboard.putNumber("Match Time",DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());

        //  //commands
        SmartDashboard.putData("Elevator Bottom Command",new ElevatorSetStageCommand(Elevator,0).alongWith(new CoralWristSetTargetPositionCommand(CoralWrist, 0)));
        SmartDashboard.putData("Elevator Intake Stage Command",new ElevatorSetStageCommand(Elevator,1).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 1)));
        SmartDashboard.putData("Elevator L2 Command",new ElevatorSetStageCommand(Elevator,2).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        SmartDashboard.putData("Elevator L3 Command",new ElevatorSetStageCommand(Elevator,3).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        SmartDashboard.putData("Elevator L4 Command",new ElevatorSetStageCommand(Elevator,4).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 3)));


        SmartDashboard.putData("Zero All Motors",new ZeroTalonCommand(Elevator.ElevatorLeft).alongWith(new ZeroTalonCommand(Elevator.ElevatorRight)).alongWith(new ZeroTalonCommand(CoralWrist.CoralWrist)));

        //subsystems
        SmartDashboard.putData("Elevator",Elevator);
        SmartDashboard.putData("Wrist",CoralWrist);
        SmartDashboard.putData("Spin",CoralSpin);
        SmartDashboard.putData("Drive Augments",DriveAugments);

    }


    @Override
    public void periodic(){

        //elevator

        if(sElevatorPIDManual.get()){
        Elevator.setTargetPosition(sElevatorPIDBar.get());
        Elevator.elevatorPID.setPID(sElevatorPIDP.get(0.2),0,sElevatorPIDD.get(0.4));
        Elevator.elevatorPID.setMaxOutput(sElevatorPIDClampUpper.get(0.4));
        Elevator.elevatorPID.setMinOutput(sElevatorPIDClampLower.get(-0.15));
        }

        pElevatorHasReached.set(Elevator.hasReachedTarget());
        pElevatorLeftMotor.set(Elevator.ElevatorLeft.getPosition().getValueAsDouble());
        pElevatorRightMotor.set(Elevator.ElevatorRight.getPosition().getValueAsDouble());


    }

}
