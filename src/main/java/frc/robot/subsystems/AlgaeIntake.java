package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.tools.PID;

public class AlgaeIntake implements Subsystem {
    CommandXboxController Manipulator;
    int AlgaeSpinId = 16;
    int AlgaeWristId = 17;
    TalonFX AlgaeSpin = new TalonFX(AlgaeSpinId);
    TalonFX AlgaeWrist = new TalonFX(AlgaeWristId);
    PID AlgaeWristPID;
    double wristTarget = 0;
    int wristPos = 0;   
    double[] wristPositions = new double[2];

    

    public AlgaeIntake (CommandXboxController c){
        Manipulator = c;

        //setting wrist positions
        wristPositions[0] = 0;
        wristPositions[1] = 300;



        //setting pid
        AlgaeWristPID = new PID(0.1,0,0.1);
        AlgaeWristPID.setGravity(0.1);
        AlgaeWristPID.setMinOutput(-0.2);
        AlgaeWristPID.setMaxOutput(0.2);


        var spinConfig = new TalonFXConfiguration();
        spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spinConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        spinConfig.Feedback.SensorToMechanismRatio = 1;
        spinConfig.Voltage.PeakForwardVoltage = 3;
        spinConfig.Voltage.PeakReverseVoltage = 3;


        var wristConfig = new TalonFXConfiguration();
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        wristConfig.Feedback.SensorToMechanismRatio = 100;
        wristConfig.Voltage.PeakForwardVoltage = 3;
        wristConfig.Voltage.PeakReverseVoltage = 3;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =false;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =false;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -10;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10;

        
        AlgaeSpin.getConfigurator().apply(spinConfig);
        AlgaeWrist.getConfigurator().apply(wristConfig);

        AlgaeWrist.setPosition(0);


        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();
        Trigger a = Manipulator.a();
        Trigger x = Manipulator.x();



        this.setDefaultCommand(Commands.run(()->{


            AlgaeWristPID.setSetPoint(wristTarget);

            AlgaeWrist.set(AlgaeWristPID.updatePID(AlgaeWrist.getPosition().getValueAsDouble())); 
            

        },this));   

         //spinning commands
        leftBumper.whileTrue(Commands.run(()->{
            AlgaeSpin.set(0.5);
        }));
        leftBumper.whileTrue(Commands.run(()->{
                AlgaeSpin.set(-0.5);
        }));
        leftBumper.and(rightBumper.whileFalse(Commands.run(()->{
            AlgaeSpin.set(0);
        })));

        //setting wrist target
        a.onTrue(Commands.runOnce(()->{
            swapTarget();
        }));
        //manual
        
        x.whileTrue(Commands.run(()->{

            AlgaeWrist.set(-Manipulator.getLeftY() * 0.25);
            wristTarget = -Manipulator.getLeftY() * 0.25;
        },this));
    }

    public void swapTarget(){

        switch(wristPos){

            case(0) -> {
                wristTarget = wristPositions[1];
                wristPos = 1;
            }
            case(1) -> {
                wristTarget = wristPositions[0];
                wristPos = 0;
            }
            default -> {
                wristTarget = wristPositions[0];
                wristPos = 0;
            }

        }
    }

    
    public void smartDashboard(){
        SmartDashboard.putNumber("Algae Wrist Position",AlgaeWrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Algae Wrist Output",AlgaeWrist.get());
        SmartDashboard.putString("Algae Wrist PID",AlgaeWristPID.toString());
        SmartDashboard.putNumber("Algae Spin Output", AlgaeSpin.get());
        SmartDashboard.updateValues();
    }

    @Override
    public void periodic(){
        smartDashboard();
    }

}
