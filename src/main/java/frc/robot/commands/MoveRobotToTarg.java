package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class MoveRobotToTarg extends Command {
    Vision Vision;
    Rotation2d RotError;
    Translation2d PoseError;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final PIDController XController = new PIDController(1, 0, 0);
    private final PIDController YController = new PIDController(1, 0, 0);
    private final PIDController turnController = new PIDController(2, 0, 0);

    public MoveRobotToTarg(PhotonTrackedTarget target, CommandSwerveDrivetrain s, CommandXboxController d){
        double RotError = Vision.CalcRotation(target).getRadians();
        PoseError = Vision.CalcTranslation();


        double dx = PoseError.getX();
        double dy = PoseError.getY();


        double vx = XController.calculate(dx, 0);
        double vy = YController.calculate(dy, 0);
        double omega = turnController.calculate(RotError, 0);

        SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric().withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
        
        drivetrain.setControl(driveRequest);
    }
}