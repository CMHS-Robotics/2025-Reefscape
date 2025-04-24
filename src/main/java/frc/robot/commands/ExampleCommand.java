package frc.robot.commands;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



//to create a command, create a class that extends the Command class
public class ExampleCommand extends Command{
    

    //create any kind of motors, controllers, etc
    TalonFX ExampleMotor;
    CommandXboxController ExampleController;


    public ExampleCommand(SubsystemBase ExampleSubsystem, TalonFX ExampleMotor, CommandXboxController ExampleController){

        //include a motor in the parameters to define what motor you control when the command is created
        this.ExampleMotor = ExampleMotor; 

        //include a controller in your paremeters to define what controller the command will use
        this.ExampleController = ExampleController;



        //include a subsystem in the constructor parameters and use addRequirements to tell the command scheduler what subsystems this command requires 
        addRequirements(ExampleSubsystem);
    }


    //What the command does when it is initially scheduled, before running-only runs once
    @Override
    public void initialize(){
        //set the motor to not move
        ExampleMotor.set(0);

        //create a new configuration for motor outputs
        MotorOutputConfigs ExampleConfig = new MotorOutputConfigs();
        //set the motor to brake when not powered
        ExampleConfig.NeutralMode = NeutralModeValue.Brake;
        //apply configuration
        ExampleMotor.getConfigurator().apply(ExampleConfig);
    }


    //What the command does every scheduler instance (every ~ 20 ms)
    @Override
    public void execute(){
        //set motor speed to the axis of the left controller stick
        double ControllerLeftAxis =  ExampleController.getLeftY();
        ExampleMotor.set(ControllerLeftAxis);
    }

    //What the command does when it ends
    @Override       //interrupted means it got cancelled or overwritten by another command
    public void end(boolean interrupted){
        //stop motor movement
        ExampleMotor.set(0);
    }

    //Stop the command when a condition is reached
    @Override
    public boolean isFinished() {
        //command never finishes on it's own
        return false;
    }






}
