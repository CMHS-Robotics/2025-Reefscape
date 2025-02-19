package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeIntake implements Subsystem {
    CommandXboxController Manipulator;
    int AlgaeMotorId = 16;
    TalonFX AlgaeMotor = new TalonFX(AlgaeMotorId);

    

    public AlgaeIntake (CommandXboxController x){
        Manipulator = x;

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        AlgaeMotor.getConfigurator().apply(config);

        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();


        
        this.setDefaultCommand(Commands.run(()->{



        },this));   

         //spinning commands
         leftBumper.whileTrue(Commands.run(()->{
            AlgaeMotor.set(0.5);
        }));
        leftBumper.whileTrue(Commands.run(()->{
                AlgaeMotor.set(-0.5);
        }));
        leftBumper.and(rightBumper.whileFalse(Commands.run(()->{
            AlgaeMotor.set(0);
        })));
    }
}
