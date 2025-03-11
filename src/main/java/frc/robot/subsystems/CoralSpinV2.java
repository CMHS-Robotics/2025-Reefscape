package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralSetSpinSpeedCommandV2;

public class CoralSpinV2 implements Subsystem {
    private final int CoralIntakeMotorA = 19;
    private final int CoralIntakeMotorB = 20;

    public TalonFX CoralIntakeRight = new TalonFX(CoralIntakeMotorA);
    public TalonFX CoralIntakeLeft = new TalonFX(CoralIntakeMotorB);   
    public CommandXboxController Manipulator;
    public CoralSpinV2(CommandXboxController x){
        Manipulator = x;
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        CoralIntakeRight.getConfigurator().apply(config);
        CoralIntakeRight.setPosition(0);

        CoralIntakeLeft.getConfigurator().apply(config);
        CoralIntakeLeft.setPosition(0);

        CoralIntakeLeft.set(0);
        CoralIntakeRight.set(0);

        //commands
        CoralSetSpinSpeedCommandV2 CoralInSpin = new CoralSetSpinSpeedCommandV2(this,0.2);
        CoralSetSpinSpeedCommandV2 CoralOutSpin = new CoralSetSpinSpeedCommandV2(this,-0.2);
        CoralSetSpinSpeedCommandV2 CoralNoSpin = new CoralSetSpinSpeedCommandV2(this,0);

        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();

        this.setDefaultCommand(CoralNoSpin);

        leftBumper.whileTrue(CoralOutSpin);

        rightBumper.whileTrue(CoralInSpin);
    }
}