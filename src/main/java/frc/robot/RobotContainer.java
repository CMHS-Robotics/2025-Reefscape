// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CoralSetSpinSpeedCommandV2;
import frc.robot.commands.CoralWristSetTargetPositionCommand;
import frc.robot.commands.ElevatorSetStageCommand;
import frc.robot.commands.MoveRobotToTarg;
import frc.robot.commands.ZeroTalonCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSpinV2;
import frc.robot.subsystems.CoralWristV2;
import frc.robot.subsystems.DriveAugments;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;
import frc.robot.subsystems.Vision.MODE;
import frc.robot.tools.DashboardSuite;
import frc.robot.subsystems.Vision.CVState;

public class RobotContainer {   


    

    private final SendableChooser<Command> autoChooser;
    public static double SpeedMultiplier = 1;
    public static double RotationSpeedMultiplier = 1;
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final Telemetry logger = new Telemetry(MaxSpeed);
    public static CommandXboxController Driver = new CommandXboxController(0);
    public static CommandXboxController Manipulator = new CommandXboxController(1);
    //define all the subsystems
    CoralSpinV2 CoralSpin = new CoralSpinV2(Manipulator);
    CoralWristV2 CoralWrist = new CoralWristV2(Manipulator);
    Elevator Elevator = new Elevator(Manipulator,CoralWrist);
    DriveAugments Augment = new DriveAugments(Driver,Elevator);

    //define all the commands that will be used in autonomous
    ElevatorSetStageCommand TopStage = new ElevatorSetStageCommand(Elevator,4);
    ElevatorSetStageCommand IntakeStage = new ElevatorSetStageCommand(Elevator,1);
    ElevatorSetStageCommand BottomStage = new ElevatorSetStageCommand(Elevator,0);
    CoralWristSetTargetPositionCommand TopCoral = new CoralWristSetTargetPositionCommand(CoralWrist,-5.78);
    CoralWristSetTargetPositionCommand IntakeCoral = new CoralWristSetTargetPositionCommand(CoralWrist,1);
    CoralWristSetTargetPositionCommand BottomCoral = new CoralWristSetTargetPositionCommand(CoralWrist,0);
    CoralSetSpinSpeedCommandV2 CoralIn = new CoralSetSpinSpeedCommandV2(CoralSpin,-0.3);
    CoralSetSpinSpeedCommandV2 CoralOut = new CoralSetSpinSpeedCommandV2(CoralSpin,0.3);

    //create drivetrain
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    //dashboard and vision subsystems
    Vision Vision = new Vision(drivetrain,Driver);
    DashboardSuite Dashboard = new DashboardSuite(Elevator, CoralSpin, CoralWrist, Vision);

    //define move to targ command
    MoveRobotToTarg moveRobotToTarg = new MoveRobotToTarg(Vision.getTarget(CAMERA.FRONT), drivetrain, Driver);

    // //target pose for a pathfinding command (i used this to return to the starting point when we were testing autonomous)
    // Pose2d targetPose = new Pose2d(7.568, 7.62, Rotation2d.fromDegrees(0));

    // // Create the constraints to use while pathfinding
    // PathConstraints constraints = new PathConstraints(
    //         3.0, 1.0,
    //         Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    // // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // Command pathfindingCommand = AutoBuilder.pathfindToPose(
    //         targetPose,
    //         constraints,
    //         0.0 // Goal end velocity in meters/sec
    // );


    public RobotContainer() {
        //start cameras
        CameraServer.startAutomaticCapture();

        //reset elevator position (just to be sure)
        Elevator.ElevatorRight.setPosition(0);
        Elevator.ElevatorLeft.setPosition(0);

        //auto commands and events
            //set topstage command to run followed by topcoral when the command TopStage is called in pathplanner
        NamedCommands.registerCommand("TopStage", TopStage.andThen(TopCoral));
            //set intake command to run at the same time as intakecoral when the command TopStage is called in pathplanner
        NamedCommands.registerCommand("IntakeStage", IntakeStage.alongWith(IntakeCoral));
        NamedCommands.registerCommand("BottomStage", BottomStage.alongWith(BottomCoral));
            //set CoralIn command to run for three seconds when the CoralIn command is called in pathplanner
        NamedCommands.registerCommand("CoralIn", CoralIn.withTimeout(3));
        NamedCommands.registerCommand("CoralOut", CoralOut.withTimeout(1));
        //NamedCommands.registerCommand("MoveRobotToTarg", moveRobotToTarg);


        //create the autochooser and put it in smartdashboard
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.updateValues();
        SmartDashboard.putData("Auto Mode", autoChooser);

        //create all the drivetrain commands
        
        configureBindings();

        //controls


        /*
          Driver:

            d pad gives precise orthagonal and diagonal control
            left stick is drive
            right stick is turn
            left bumper is reset field orientation
            b is brake 
            left trigger is slow
            right trigger is super slow          
         
         */


        /*Manipulator
        

        D-pad: elevator levels
        down-intake stage
        left-stage 1
        up-stage 2
        right-stage 3

        b brings it back to the bottom



        hold right trigger to engage manual elevator control with right stick
        will update the pid target position, so after disengaging control, elevator will stay


        hold left trigger to engage manual wrist control with left stick
        will currently NOT update target pid, so after disengaging manual control, will snap back to the posiition correlating to the elevator level

        click in right stick to reset elevator encoder to 0

        click in left stick to reset wrist encoder to 0




        */
    }

    private double vectorDot(){
        Vector<N2> controllerVector = VecBuilder.fill(-Driver.getLeftY(),-Driver.getLeftX());
        Vector<N2> targetVector = VecBuilder.fill(Math.cos(targetRadians()),Math.sin(targetRadians()));
        
        return controllerVector.dot(targetVector);
    }

    private double targetRadians(){
        return Vision.getTargetPose(Vision.getTarget(CAMERA.FRONT)).getRotation().toRotation2d().getRadians();
    }


    //Note
    //Use Above Functions, Get Robot Current Position
    //Can use d pad.
    //Running Take:
    //Use x to enter/exit mode, set state to search for targ
    //Search Target, If Not Found, Rotate Until, Execute Move To Targ.
    //When At Targ, Allow LR selection.
    //Make A Que For Actions, So Driver Enters Mode And Immediatly Presses Left D pad, store this and execute when ready. Allow Changing Upon Final State Or In Between 
    //:Always Be Listening For LR Input


    private double leftStickPolarity(boolean x){
        return (x)?Driver.getLeftX()/Math.abs(Driver.getLeftX()):Driver.getLeftY()/Math.abs(Driver.getLeftY());
    }

    private double getXStrafe(){
        if(Driver.rightBumper().getAsBoolean()&&Vision.getCurrentMode().equals(MODE.LOCKVECTOR)){
            return (Vision.xTranslatePID.updatePID(Vision.getRobotPose().getX())/* + (Math.cos(targetRadians())) * vectorDot() * -leftStickPolarity(false)*/)  * SpeedMultiplier;
        }
        return -Driver.getLeftY() * MaxSpeed * SpeedMultiplier;
    }

    private double getYStrafe(){
        if(Driver.rightBumper().getAsBoolean()&&Vision.getCurrentMode().equals(MODE.LOCKVECTOR)){
            return (Vision.yTranslatePID.updatePID(Vision.getRobotPose().getY())/* + (Math.sin(targetRadians())) * vectorDot()* -leftStickPolarity(true)*/)  * SpeedMultiplier;
        }
        return -Driver.getLeftX() * MaxSpeed * SpeedMultiplier;
    }

    private double getTurn(){
        if(Driver.rightBumper().getAsBoolean()&&!Vision.getCurrentMode().equals(MODE.YAWTARGET)){
            return Vision.turnTrackingPID.updatePID(Vision.getRobotPose().getRotation().getDegrees()) * MaxAngularRate * RotationSpeedMultiplier;
        }
        if(Driver.rightBumper().getAsBoolean()&&Vision.getCurrentMode().equals(MODE.YAWTARGET)){
            return -Vision.turnTrackingPID.updatePID(0) * MaxAngularRate * RotationSpeedMultiplier;
        }
        return -Driver.getRightX() * MaxAngularRate * RotationSpeedMultiplier;
    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(getXStrafe())
                    .withVelocityY(getYStrafe())
                    .withRotationalRate(getTurn())
            )
            
        );


        //d pad precise positioning
        Driver.povDown().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-1 * MaxSpeed * SpeedMultiplier  )));
        Driver.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(1 * MaxSpeed * SpeedMultiplier  )));
        Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(1 * MaxSpeed * SpeedMultiplier  )));
        Driver.povRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-1 * MaxSpeed * SpeedMultiplier  )));

        //LockOnMechanism
        

        Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  ).withVelocityY(1/Math.sqrt(2)*MaxSpeed*SpeedMultiplier  )));
        Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  ).withVelocityY(-1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  )));
        Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  ).withVelocityY(1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  )));
        Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  ).withVelocityY(-1/Math.sqrt(2) * MaxSpeed * SpeedMultiplier  )));

        Driver.x().onTrue(moveRobotToTarg);

        Driver.b().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
        ));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        //Driver.back().onTrue(pathfindingCommand);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    
    public void zeroMotors(){
        new ZeroTalonCommand(Elevator.ElevatorLeft).alongWith(new ZeroTalonCommand(Elevator.ElevatorRight)).alongWith(new ZeroTalonCommand(CoralWrist.CoralWrist)).schedule();
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
