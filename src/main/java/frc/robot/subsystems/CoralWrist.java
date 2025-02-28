package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.tools.PID;

public class CoralWrist implements Subsystem {
    CommandXboxController Manipulator;
    int CoralWristMotorId = 15;
    public SparkMax CoralWrist = new SparkMax(CoralWristMotorId,SparkMax.MotorType.kBrushless);
    RelativeEncoder WristEncoder = CoralWrist.getEncoder();
    double wristTarget;
    PID coralWristPID;
    Elevator elevator;
    int elevatorStage = 0;
    double gravity = 0;
    double[] wristPositions = new double[4];

    

    public CoralWrist (CommandXboxController c,Elevator e){
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
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
        wristConfig.encoder
        .positionConversionFactor(100)
        .velocityConversionFactor(100);
        wristConfig.softLimit
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-800)
        .forwardSoftLimit(0);

        CoralWrist.configure(wristConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        WristEncoder.setPosition(0);

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


                CoralWrist.set(coralWristPID.updatePID(WristEncoder.getPosition())); 
                

        },this));   

        //reset encoder
        leftStickClick.onTrue(Commands.runOnce(()->{
            WristEncoder.setPosition(0);
        }));

        //manual control
        leftTrigger.whileTrue(Commands.run(()->{

            CoralWrist.set(-Manipulator.getLeftY() * 0.05);
        },this));


    }


    public void smartDashboard(){
        SmartDashboard.putNumber("Coral Wrist Encoder Position",WristEncoder.getPosition());
        SmartDashboard.putNumber("Coral Wrist Output",CoralWrist.get());
        SmartDashboard.putString("Coral Wrist PID",coralWristPID.toString());
        SmartDashboard.putNumber("gravity calc",getGravity());
        SmartDashboard.updateValues();
    }

    public double getGravity(){
        double RadiansPerTick = Math.PI * 2 / 1200;
        double angle = WristEncoder.getPosition() * RadiansPerTick;
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
