package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralWristPIDTargetCommand;
import frc.robot.commands.ZeroTalonCommand;
import frc.robot.tools.PID;

public class CoralWristV2 extends SubsystemBase {
    CommandXboxController Manipulator;
    int CoralWristMotorId = 15;
    public TalonFX CoralWrist = new TalonFX(CoralWristMotorId);
    double wristTarget = 0;
    public final PID coralWristPID;
    double gravity = 0;
    public double[] wristPositions = new double[4];
    boolean shuffleboardManualControl = false;
    double shuffleboardManualControlValue = 0.0;
    boolean reachedTarget;
    

    public CoralWristV2 (CommandXboxController c){
        Manipulator = c;

        //set wrist positions
        wristPositions[0] = 0;
        wristPositions[1] = -1.9;
        wristPositions[2] = -5.1;
        wristPositions[3] = -5.85;


        CoralWrist.setPosition(0);

        //set pid
        coralWristPID = new PID(0.2,0,0.1);
        coralWristPID.setMaxOutput(0.1);
        coralWristPID.setMinOutput(-0.1);
        coralWristPID.setReachedTargetErrorThreshold(0.3);

        //motor configs
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        var motOutConfig = wristConfig.MotorOutput;
        motOutConfig.Inverted = InvertedValue.Clockwise_Positive;
        motOutConfig.NeutralMode = NeutralModeValue.Brake;
        //wristConfig.Feedback.SensorToMechanismRatio = 1/1000;
        //wristConfig.Feedback.RotorToSensorRatio = 100;


        CoralWrist.getConfigurator().apply(wristConfig);
        

        //triggers
        Trigger leftTrigger = Manipulator.leftTrigger();
        Trigger leftStickClick = Manipulator.leftStick();

        //commands
        CoralWristPIDTargetCommand pidTarget = new CoralWristPIDTargetCommand(this, coralWristPID);


        //default command
        this.setDefaultCommand(pidTarget);   

        //reset encoder
        leftStickClick.onTrue(new ZeroTalonCommand(CoralWrist));

        //manual control
        leftTrigger.whileTrue(Commands.run(()->{
            CoralWrist.set(-Manipulator.getLeftY() * 0.05);
        },this));


    }

    public void setShuffleboardManualControl(boolean s){
        shuffleboardManualControl = s;
    }
    
    public boolean getShuffleboardManualControl(){
        return shuffleboardManualControl;
    }

    public void setShuffleboardManualControlValue(double s){
        shuffleboardManualControlValue = s;
    }
    
    public double getShuffleboardManualControlValue(){
        return shuffleboardManualControlValue;
    }

    
    public void setTarget(double t){
        wristTarget = t;
    }

    public double getTarget(){
        return wristTarget;
    }

    public void setTargetLevel(int a){
        wristTarget = wristPositions[a];
    }
    
    public boolean hasReachedTarget(){
        reachedTarget = (Math.abs(coralWristPID.getError()) < coralWristPID.getReachedTargetErrorThreshold());
        return reachedTarget;
    }
    public void setReachedTarget(boolean g){
        reachedTarget = g;
    }


    public void smartDashboard(){
        SmartDashboard.putNumber("Coral Wrist Encoder Position",CoralWrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Coral Wrist Output",CoralWrist.get());
        SmartDashboard.putString("Coral Wrist PID",coralWristPID.toString());
        SmartDashboard.putNumber("gravity calc",getGravity());
        SmartDashboard.putBoolean("Coral Wrist has reached target",hasReachedTarget());
        SmartDashboard.updateValues();
    }

    public double getGravity(){
        double RadiansPerTick = Math.PI * 2 / 1200;
        double angle = CoralWrist.getPosition().getValueAsDouble() * RadiansPerTick;
        double weight = 5;
        double cx = 0.3;
        double torque = 9.8 * cx * weight * Math.cos(angle);
        SmartDashboard.putNumber("Torque",torque);
        SmartDashboard.putString("Angle",angle / Math.PI + " Pi");
        double torqueCompensation = 0.2;
        gravity = torque * torqueCompensation;
        return gravity;
    }

    @Override
    public void periodic(){
        smartDashboard();
    }



}
