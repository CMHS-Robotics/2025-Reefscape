package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.jni.CANSparkJNI;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralIntake implements Subsystem {
    CommandXboxController Manipulator;
    int CoralContinuousMotorId = 15;
    TalonFX CoralContinuousMotor = new TalonFX(CoralContinuousMotorId);
    CANSparkJNI NeoWristMotor = new CANSparkJNI();
    

    public CoralIntake (CommandXboxController x){
        Manipulator = x;

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        CoralContinuousMotor.getConfigurator().apply(config);

        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();
        this.setDefaultCommand(Commands.run(()->{

            if(leftBumper.getAsBoolean()){
                CoralContinuousMotor.set(0.5);
            }else if(rightBumper.getAsBoolean()){
                CoralContinuousMotor.set(-0.5);
            }else{
                CoralContinuousMotor.set(0);
            }

        },this));   
    }
}
