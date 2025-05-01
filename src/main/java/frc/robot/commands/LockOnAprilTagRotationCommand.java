package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;

public class LockOnAprilTagRotationCommand extends Command {
    
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
    boolean skip = false;

    public LockOnAprilTagRotationCommand(CommandSwerveDrivetrain s, Vision v, CommandXboxController x){
        swerve = s;
        vision = v;
        Driver = x;
        idTarget = false;
        addRequirements(swerve);
    }
    
    public LockOnAprilTagRotationCommand(CommandSwerveDrivetrain s, Vision v,CommandXboxController x, int id){
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

        skip = !vision.hasTarget(CAMERA.FRONT);

        if(!skip){
            target = vision.getTarget(CAMERA.FRONT);
            targetYaw = target.getYaw();
            vision.turnTrackingPID.setSetPoint(targetYaw);

            SmartDashboard.putNumber("Target Yaw", targetYaw);

            SpeedMultiplier = RobotContainer.SpeedMultiplier;
            swerve.applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(-Driver.getLeftY() * MaxSpeed * SpeedMultiplier)
            .withVelocityY(-Driver.getLeftX() * MaxSpeed * SpeedMultiplier  )
            .withRotationalRate(-1.0 * vision.turnTrackingPID.updatePID(0) * MaxAngularRate));
        }

    
    }
}
