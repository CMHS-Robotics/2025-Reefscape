package frc.robot.tools;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
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
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;

public class DashboardSuite extends SubsystemBase{
    
    //subsystems
    Elevator Elevator;
    CoralSpinV2 CoralSpin;
    CoralWristV2 CoralWrist;
    DriveAugments DriveAugments;
    CommandSwerveDrivetrain Swerve;
    Vision Vision;

    //subscribers
        //elevator
    DoubleSubscriber sElevatorPIDBar;
    BooleanSubscriber sElevatorPIDManual;
    DoubleSubscriber sElevatorPIDP;
    DoubleSubscriber sElevatorPIDD;
    DoubleSubscriber sElevatorPIDClampUpper;
    DoubleSubscriber sElevatorPIDClampLower;
        //vision
    IntegerSubscriber sVisionTarget;


    //publishers
        //elevator
    DoublePublisher pElevatorLeftMotor;
    DoublePublisher pElevatorRightMotor;
    StringPublisher pElevatorPID;
    StringPublisher pElevatorPIDSettings;
    BooleanPublisher pElevatorHasReached;
    DoublePublisher pElevatorPIDResult;
    DoublePublisher pElevatorPIDPreClampResult;
        //coral
    DoublePublisher pCoralWristMotor;
    DoublePublisher pCoralWristOutput;
    StringPublisher pCoralWristPID;
    BooleanPublisher pCoralWristHasReached;
        //vision
    BooleanPublisher pVisionHasDetected;
    IntegerPublisher pAprilTagDetected;
    BooleanPublisher pVisionHasTarget;
    
    public DashboardSuite(Elevator e, CoralSpinV2 s, CoralWristV2 w,Vision v){
        Elevator = e;
        CoralSpin = s;
        CoralWrist = w;
        Vision = v;
        initialize();
    }

    public final void initialize(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        //elevator data
        NetworkTable ElevatorData = inst.getTable("Elevator");


        pElevatorLeftMotor = ElevatorData.getDoubleTopic("Elevator Left Motor").publish();
        pElevatorRightMotor = ElevatorData.getDoubleTopic("Elevator Right Motor").publish();
        pElevatorPID = ElevatorData.getStringTopic("Elevator PID").publish();
        pElevatorPIDSettings = ElevatorData.getStringTopic("Elevator PID Settings").publish();
        pElevatorPIDResult = ElevatorData.getDoubleTopic("Elevator PID Result").publish();
        pElevatorPIDPreClampResult = ElevatorData.getDoubleTopic("Elevator PID Pre Clamp Result").publish();
        pElevatorHasReached = ElevatorData.getBooleanTopic("Elevator Has Reached Target").publish();

        NetworkTable ElevatorManualPIDTargetting = inst.getTable("Elevator PID Manual");

        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Bar Manual Target").publish().set(0.0);
        ElevatorManualPIDTargetting.getBooleanTopic("Elevator Manual Control Active").publish().set(false);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID P Value").publish().set(0.2);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID D Value").publish().set(0.4);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Lower Limit").publish().set(-0.15);
        ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Upper Limit").publish().set(0.4);


        sElevatorPIDBar = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Bar Manual Target").subscribe(0.0);
        sElevatorPIDManual = ElevatorManualPIDTargetting.getBooleanTopic("Elevator Manual Control Active").subscribe(false);
        sElevatorPIDP = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID P Value").subscribe(0);
        sElevatorPIDD = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID D Value").subscribe(0);
        sElevatorPIDClampLower = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Lower Limit").subscribe(0);
        sElevatorPIDClampUpper = ElevatorManualPIDTargetting.getDoubleTopic("Elevator PID Upper Limit").subscribe(0);

        //coral data
        NetworkTable CoralData = inst.getTable("Coral");

        pCoralWristPID = CoralData.getStringTopic("Elevator PID").publish();
        pCoralWristMotor = CoralData.getDoubleTopic("Coral Wrist Position").publish();
        pCoralWristOutput = CoralData.getDoubleTopic("Coral Wrist Output").publish();
        pCoralWristHasReached = CoralData.getBooleanTopic("Wrist Has Reached Target").publish();

        //vision data
        NetworkTable VisionData = inst.getTable("Vision");

        VisionData.getIntegerTopic("Vision April Tag ID Target").publish().set(0);
        pAprilTagDetected = VisionData.getIntegerTopic("ID of April Tag Detected").publish();
        pVisionHasDetected = VisionData.getBooleanTopic("Vision Has Detected Target").publish();
        pVisionHasTarget = VisionData.getBooleanTopic("Vision Has Target").publish();

        sVisionTarget = VisionData.getIntegerTopic("Vision April Tag ID Target").subscribe(0);


        //other
        SmartDashboard.putNumber("Match Time",DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());

        //commands
        SmartDashboard.putData("Elevator Bottom Command",new ElevatorSetStageCommand(Elevator,0).alongWith(new CoralWristSetTargetPositionCommand(CoralWrist, 0)));
        SmartDashboard.putData("Elevator Intake Stage Command",new ElevatorSetStageCommand(Elevator,1).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 1)));
        SmartDashboard.putData("Elevator L2 Command",new ElevatorSetStageCommand(Elevator,2).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        SmartDashboard.putData("Elevator L3 Command",new ElevatorSetStageCommand(Elevator,3).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 2)));
        SmartDashboard.putData("Elevator L4 Command",new ElevatorSetStageCommand(Elevator,4).andThen(new CoralWristSetTargetPositionCommand(CoralWrist, 3)));

        //SmartDashboard.putData("Lock On April Tag",new LockOnAprilTagCommand(Swerve, Vision, RobotContainer.Driver, (int)sVisionTarget.get()));

        SmartDashboard.putData("Zero All Motors",new ZeroTalonCommand(Elevator.ElevatorLeft).alongWith(new ZeroTalonCommand(Elevator.ElevatorRight)).alongWith(new ZeroTalonCommand(CoralWrist.CoralWrist)));
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
        pElevatorPID.set(Elevator.elevatorPID.toString());
        pElevatorPIDSettings.set(Elevator.elevatorPID.getSettings());
        pElevatorPIDResult.set(Elevator.elevatorPID.getResult());
        pElevatorPIDPreClampResult.set(Elevator.elevatorPID.getResultPreClamp());


        //coral
        pCoralWristHasReached.set(CoralWrist.hasReachedTarget());
        pCoralWristMotor.set(CoralWrist.CoralWrist.getPosition().getValueAsDouble());
        pCoralWristPID.set(CoralWrist.coralWristPID.toString());
        pCoralWristOutput.set(CoralWrist.CoralWrist.get());


        //vision
        pVisionHasDetected.set(Vision.hasTarget(CAMERA.FRONT,(int)sVisionTarget.get()));
        pVisionHasTarget.set(Vision.hasTarget(CAMERA.FRONT));
        pAprilTagDetected.set((Vision.hasTarget(CAMERA.FRONT))?Vision.getTarget(CAMERA.FRONT).getFiducialId():0);

    
    }
}
