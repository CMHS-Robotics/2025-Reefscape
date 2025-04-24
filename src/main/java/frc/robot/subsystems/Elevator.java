package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralWristSetTargetPositionCommand;
import frc.robot.commands.ElevatorFreeMoveCommand;
import frc.robot.commands.ElevatorPIDTargetCommand;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.commands.ZeroTalonCommand;
import frc.robot.tools.PID;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator extends SubsystemBase {

    //set motor ids
    private final int elevatorMotorLeftId = 13;
    private final int elevatorMotorRightId = 14;


    //create variables
    private int stageLevel = 0;

    final private double[] stages = new double[5];

    private double targetPosition = 0;

    private boolean reachedTarget = false;

    //create PID
    final public PID elevatorPID;

    //create commands
    ElevatorFreeMoveCommand freeMoveCommand;
    ElevatorPIDTargetCommand targetPositionCommand;

    //create and define motors
    public TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    public TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   

    //create controller
    public CommandXboxController Manipulator;

    //create wrist
    CoralWristV2 wrist;

    public Elevator(CommandXboxController x, CoralWristV2 w){
        
        //define controller and wrist based on parameter
        Manipulator = x;
        wrist = w;
        //set PID
        elevatorPID = new PID(0.2,0,0.4);//.4,0,.52

        elevatorPID.setMaxOutput(0.4);//.24
        elevatorPID.setMinOutput(-0.15);//-.15
        elevatorPID.setGravity(0.023);
        elevatorPID.setReachedTargetErrorThreshold(1);

        //set stage levels
        stages[0]= 0;
        stages[1]=  7.9;
        stages[2]= 10.6;
        stages[3]= 16;
        stages[4]= 24.1;

        //Make new config
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        //elevator limits

        var softwarelimit = config.SoftwareLimitSwitch;
        softwarelimit.ForwardSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitThreshold = 1;
        softwarelimit.ForwardSoftLimitThreshold = 24.2;

        //apply configs and reset motors
        ElevatorLeft.getConfigurator().apply(config);
        ElevatorLeft.setPosition(0);

        ElevatorRight.getConfigurator().apply(config);
        ElevatorRight.setPosition(0);
        
        ElevatorLeft.setPosition(Rotations.of(0));
        ElevatorRight.setPosition(Rotations.of(0));

        ElevatorLeft.set(0);
        ElevatorRight.set(0);
        
        //define commands
        freeMoveCommand = new ElevatorFreeMoveCommand(this);

        targetPositionCommand = new ElevatorPIDTargetCommand(this,elevatorPID);

        this.setDefaultCommand(targetPositionCommand);


        //define triggers
        Trigger povUp = Manipulator.povUp();
        Trigger povLeft = Manipulator.povLeft();
        Trigger povRight = Manipulator.povRight();
        Trigger povDown = Manipulator.povDown();
        Trigger rightTrigger = Manipulator.rightTrigger();
        Trigger rightStickClick = Manipulator.rightStick();
        Trigger b = Manipulator.b();

        //bind commands to triggers
        b.onTrue(new ElevatorSetStageCommand(this, 0).alongWith(new CoralWristSetTargetPositionCommand(wrist, 0)));
        povDown.onTrue(new ElevatorSetStageCommand(this, 1).andThen(new CoralWristSetTargetPositionCommand(wrist, 1)));
        povLeft.onTrue(new ElevatorSetStageCommand(this, 2).andThen(new CoralWristSetTargetPositionCommand(wrist, 2)));
        povUp.onTrue(new ElevatorSetStageCommand(this,3).andThen(new CoralWristSetTargetPositionCommand(wrist, 2)));
        povRight.onTrue(new ElevatorSetStageCommand(this,4).andThen(new CoralWristSetTargetPositionCommand(wrist,3)));

        rightTrigger.whileTrue(freeMoveCommand);

        //reset motor position command
        rightStickClick.onTrue(new ZeroTalonCommand(ElevatorLeft).alongWith(new ZeroTalonCommand(ElevatorRight)));

        //resetting motors again just to bet sure
        ElevatorRight.setPosition(0,1);
        ElevatorLeft.setPosition(0,1);
    }
        
    //this never got used but we were gonna make it so that we could put in a measurement in feet and it would output how much the motor had to turn
    public final double distanceToMotorRot(double distance){
                double r2 = 0.289;
                double r3 = 1.6;
                double r5 = 1.2;
        
                double finalRots = distance / (r2*r5)/r3;
        
                return finalRots;
    }
    

    @Override
    public void periodic(){

    }

    //getters and setters for all the class variables
    public double getTargetPosition(){
        return targetPosition;
    }
    public void setTargetPosition(double t){
        targetPosition = t;
    }
    public void setTargetStage(int s){
        targetPosition = stages[s];
    }
    public int getStageLevel(){
        return stageLevel;
    }
    public void setStageLevel(int s){
        stageLevel = s;
    }

    //says the elevator has reached its target if the error is less than the configurated cutoff
    public boolean hasReachedTarget(){
        reachedTarget = (Math.abs(elevatorPID.getError()) < elevatorPID.getReachedTargetErrorThreshold());
        return reachedTarget;
    }


 }



