package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralWristSetTargetPositionCommand;
import frc.robot.commands.ElevatorFreeMoveCommand;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.commands.ElevatorTargetPositionCommand;
import frc.robot.commands.ZeroTalonCommand;
import frc.robot.tools.PID;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator extends SubsystemBase {
    private final int elevatorMotorLeftId = 13;
    private final int elevatorMotorRightId = 14;

    private int stageLevel = 0;

    final private double[] stages = new double[5];

    private double targetPosition = 0;

    private boolean reachedTarget = false;

    final public PID elevatorPID;

    ElevatorFreeMoveCommand freeMoveCommand;
    ElevatorTargetPositionCommand targetPositionCommand;

    public TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    public TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    public CommandXboxController Manipulator;

    CoralWristV2 wrist;

    public Elevator(CommandXboxController x,CoralWristV2 w){
        Manipulator = x;
        wrist = w;
        //set PID
        elevatorPID = new PID(0.4,0,0.52);

        elevatorPID.setMaxOutput(0.24);//.17
        elevatorPID.setMinOutput(-0.15);//-.1
        elevatorPID.setGravity(0.023);
        elevatorPID.setThresholdOn(false);
        elevatorPID.setErrorThreshold(0.4);
        elevatorPID.setThresholdValue(0.04);
        elevatorPID.setReachedTargetErrorThreshold(1);

        //set stage levels
        stages[0]= 0;
        stages[1]=  7.9;
        stages[2]= 10.6;
        stages[3]= 16;
        stages[4]= 24;

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

        targetPositionCommand = new ElevatorTargetPositionCommand(this,elevatorPID);

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
        
    public final double distanceToMotorRot(double distance){
                double r2 = 0.289;
                double r3 = 1.6;
                double r5 = 1.2;
        
                double finalRots = distance / (r2*r5)/r3;
        
                return finalRots;
    }
    

    private void SmartDashboard(){
        SmartDashboard.putNumber("Left Motor Pos", ElevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Pos", ElevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("PID Target Position",targetPosition);
        SmartDashboard.putNumber("Stage Level", (int)stageLevel);
        SmartDashboard.putString("Left Motor Request",ElevatorLeft.getAppliedControl().toString());
        SmartDashboard.putString("Right Motor Request",ElevatorRight.getAppliedControl().toString());
        SmartDashboard.putNumber("Joystick Right Y: ", Manipulator.getRightY());
        SmartDashboard.putNumber("Joystick Left Y: ", Manipulator.getLeftY());
        SmartDashboard.putString("PID control",elevatorPID.toString());
        SmartDashboard.putBoolean("Elevator has reached target",hasReachedTarget());
        //SmartDashboard.putData("motorrrrr",ElevatorLeft);

        // if(this.getCurrentCommand()!=null){
        //     SmartDashboard.putString("Command Running:",this.getCurrentCommand().toString());
        // }else{
        //     SmartDashboard.putString("Command Running:","null");

        // }

        SmartDashboard.putNumber("bruh why is the error off",ElevatorLeft.getPosition().getValueAsDouble() - targetPosition);
        SmartDashboard.putNumber("im crashing out",elevatorPID.getError());
        SmartDashboard.putNumber("Pid get value - pid get target",elevatorPID.getInputValue()-elevatorPID.getSetPoint());
        SmartDashboard.updateValues();
    }


    @Override
    public void periodic(){
        SmartDashboard();

    }

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

    public boolean hasReachedTarget(){
        reachedTarget = (Math.abs(elevatorPID.getError()) < elevatorPID.getReachedTargetErrorThreshold());
        return reachedTarget;
    }
    public void setReachedTarget(boolean g){
        reachedTarget = g;
    }


 }



