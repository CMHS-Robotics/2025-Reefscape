package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionV2;

public class MoveToTagCommand extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final VisionV2 vision;

    private final SwerveRequest zero = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    // PID controllers
    private final PIDController KPForward = new PIDController(0.3, 0.0, 0.00);
    private final PIDController KPSideways = new PIDController(0.3, 0.0, 0.00);
    private final PIDController KPRotation = new PIDController(0.3, 0.0, 0.0);

    // Tag dropout protection
    private int framesWithoutTag = 0;
    private static final int MAX_FRAME_LOSS = 5;   // tolerate ~5 frames of dropout

    public MoveToTagCommand(CommandSwerveDrivetrain swerve, VisionV2 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);

        // Configure PID tolerances
        KPForward.setTolerance(0.05);     // 5 cm
        KPSideways.setTolerance(0.05);    // 5 cm
        KPRotation.setTolerance(0.05);    // rad (~3 deg)
    }

    @Override
    public void execute() {
        Transform3d transform = vision.getTagTransform();

        if (transform == null) {
            framesWithoutTag++;

            // Stop ONLY if tag is fully lost for too long
            if (framesWithoutTag >= MAX_FRAME_LOSS) {
                swerve.setControl(zero);
            }
            return;
        }

        // Tag is visible → reset counter
        framesWithoutTag = 0;

        double OffSetConstant = (Math.PI/180)*20;

        // Robot position relative to tag
        double forwardError = -transform.getX();
        double sidewaysError = -transform.getY();
        double yawError = -(transform.getRotation().toRotation2d().getRadians())+OffSetConstant;

        // Calculate PID outputs
        double vx = KPForward.calculate(forwardError, 0);
        double vy = KPSideways.calculate(sidewaysError, 0);
        double omega = KPRotation.calculate(yawError, 0);

        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);

        swerve.setControl(request);
    }

    @Override
    public boolean isFinished() {
        Transform3d transform = vision.getTagTransform();
        if (transform == null) return false;  // dropout → keep running

        // Physical closeness check
        boolean closeEnough =
            Math.abs(transform.getX()) < 0.2 &&   // within 20 cm forward
            Math.abs(transform.getY()) < 0.1;     // within 10 cm sideways

        // PID stability check
        boolean pidSettled =
            KPForward.atSetpoint() &&
            KPSideways.atSetpoint() &&
            KPRotation.atSetpoint();

        return closeEnough && pidSettled;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot fully
        swerve.setControl(zero);
    }
}