package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;

public class LockOnAprilTagCommand extends Command {
    
    CommandSwerveDrivetrain swerve;
    Vision vision;
    CommandXboxController Driver;
    boolean targetVisible;
    int id;
    double MaxSpeed = RobotContainer.MaxSpeed;
    double MaxAngularRate = RobotContainer.MaxAngularRate;
    double SpeedMultiplier = 1;
    boolean idTarget = false;
    PhotonTrackedTarget target;
    double targetYaw;

    public LockOnAprilTagCommand(CommandSwerveDrivetrain s, Vision v, CommandXboxController x){
        swerve = s;
        vision = v;
        Driver = x;
        idTarget = false;
        addRequirements(swerve);
    }
    
    public LockOnAprilTagCommand(CommandSwerveDrivetrain s, Vision v,CommandXboxController x, int id){
        swerve = s;
        vision = v;
        Driver = x;
        idTarget = true;
        this.id = id;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        if(idTarget){
            if(!vision.hasTarget(CAMERA.FRONT,id)){
                this.cancel();
            }
            target = vision.getTarget();
        }else{
            if(!vision.hasTarget(CAMERA.FRONT)){
                this.cancel();
            }
            target = vision.getResults().get(0).getBestTarget();
        }      

        targetYaw = target.getYaw();
        vision.turnTrackingPID.setSetPoint(targetYaw);

        SpeedMultiplier = RobotContainer.SpeedMultiplier;
        swerve.applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(-Driver.getLeftY() * MaxSpeed * SpeedMultiplier)
        .withVelocityY(-Driver.getLeftX() * MaxSpeed * SpeedMultiplier  )
        .withRotationalRate(-1.0 * vision.turnTrackingPID.updatePID(0) * MaxAngularRate));
    }

    

}
