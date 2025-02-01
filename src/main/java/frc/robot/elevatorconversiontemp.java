import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;

public class elevatorconversiontemp {
    public static final Angle distanceToMotorRot(double distance){
        double r2 = 0.289;
        double r3 = 1.6;
        double r5 = 1.2;

        double finalRad = (2*Math.PI*r2*r5)/r3;
    }
}
