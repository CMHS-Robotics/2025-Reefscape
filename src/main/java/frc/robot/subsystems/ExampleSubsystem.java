package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExampleCommand;


//To create a Subsystem, extend SubsystemBase
public class ExampleSubsystem extends SubsystemBase {
    
    //define motors, controllers, commands, etc
    TalonFX ExampleMotor = new TalonFX(0);
    TalonFX ExampleMotor2 = new TalonFX(1);
    CommandXboxController ExampleController;
    ExampleCommand ExampleCommand;
    ExampleCommand ExampleDefaultCommand;



    public ExampleSubsystem(CommandXboxController ExampleController){
        //include controller parameter in constructor so controller object can be defined in robotContainer and passed as an arugment when subsystem is initialized
        this.ExampleController = ExampleController;


        //define commands
            //this command will control ExampleMotor using ExampleController
        ExampleCommand = new ExampleCommand(this,ExampleMotor,ExampleController);
            //this command will control ExampleMotor2 using ExampleController
        ExampleDefaultCommand = new ExampleCommand(this,ExampleMotor2,ExampleController);


        //create command triggers from controller inputs
        Trigger AButton = ExampleController.a();

        //bind commands to triggers
            //While A button is pressed, ExampleCommand will be scheduled
        AButton.whileTrue(ExampleCommand);

        //set default command
            //this command will run while the subsystem is not in use
            //*****Default Commands MUST require the subsystem (addrequirements) that is setting it as their default****** 
        setDefaultCommand(ExampleDefaultCommand);


        //this subsystem will run ExampleDefaultCommand and control ExampleMotor2 while the a button is not held
        //while A is held, the controller will control ExampleMotor instead


        //if all functionality is handeled entirely with commands, the scheduler entirely handles it all and you do not need periodic functions querying input
    }


    //this will run every scheduler run (every ~20 ms)
    @Override
    public void periodic(){

    }





}
