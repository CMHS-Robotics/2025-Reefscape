package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.tools.PID;

public class CoralWristV2 implements Subsystem {
    CommandXboxController Manipulator;
    int CoralWristMotorId = 15;
    public TalonFX CoralWrist = new TalonFX(CoralWristMotorId);
    double wristTarget;
    PID coralWristPID;
    Elevator elevator;
    int elevatorStage = 0;
    double gravity = 0;
    double[] wristPositions = new double[4];

    

    public CoralWristV2 (CommandXboxController c,Elevator e){
        Manipulator = c;
        elevator = e;

        //set wrist positions
        wristPositions[0] = 0;
        wristPositions[1] = -186;
        wristPositions[2] = -560;
        wristPositions[3] = -520;


        //set pid
        coralWristPID = new PID(0.0007,0,0.0003);
        coralWristPID.setMaxOutput(0.1);
        coralWristPID.setMinOutput(-0.1);
        coralWristPID.setGravity(0.01);


        //motor configs
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        var motOutConfig = wristConfig.MotorOutput;
        motOutConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        motOutConfig.NeutralMode = NeutralModeValue.Brake;
        wristConfig.Feedback.SensorToMechanismRatio = 100;


        CoralWrist.getConfigurator().apply(wristConfig);
        

        //triggers
        Trigger leftTrigger = Manipulator.leftTrigger();
        Trigger leftStickClick = Manipulator.leftStick();


        //default command
        this.setDefaultCommand(Commands.run(()->{


            
                elevatorStage = elevator.getStageLevel();
                
                wristTarget = switch (elevatorStage) {
                    case 0 -> wristPositions[0];
                    case 1 -> wristPositions[1];
                    case 2 -> wristPositions[2];
                    case 3 -> wristPositions[2];
                    case 4 -> wristPositions[3];
                    default -> wristPositions[0];
                };

                //coralWristPID.setGravity(getGravity());
                coralWristPID.setSetPoint(wristTarget);


                CoralWrist.set(coralWristPID.updatePID(CoralWrist.getPosition().getValueAsDouble())); 
                

        },this));   

        //reset encoder
        leftStickClick.onTrue(Commands.runOnce(()->{
            CoralWrist.setPosition(0);
        }));

        //manual control
        leftTrigger.whileTrue(Commands.run(()->{

            CoralWrist.set(-Manipulator.getLeftY() * 0.05);
        },this));


    }


    public void smartDashboard(){
        SmartDashboard.putNumber("Coral Wrist Encoder Position",CoralWrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Coral Wrist Output",CoralWrist.get());
        SmartDashboard.putString("Coral Wrist PID",coralWristPID.toString());
        SmartDashboard.putNumber("gravity calc",getGravity());
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
