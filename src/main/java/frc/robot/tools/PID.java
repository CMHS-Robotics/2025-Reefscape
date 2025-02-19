package frc.robot.tools;

public class PID {
	private double error;
	private double totalError;
	private double prevError;

	private double PValue;
	private double IValue;
	private double DValue;

	private double gravity;

	// Dictates the inputs and outputs
	private double maxInput;
	private double minInput;
	private double maxOutput = 1500;// defaults to 100% and -100% motor power
	private double minOutput = -1500;

	private boolean continuous = false; // only for absolute encoders
	private double setPoint; // this will be set continuously
	private double result;

	private boolean thresholdOn = false;
	private double errorThreshold = 0;
	private double thresholdValue = 0;

	public PID(double kp, double ki, double kd) {
		PValue = kp;
		IValue = ki;
		DValue = kd;
	}

	/**
	 * Updates the PID controller with the current sensor value and calculates the
	 * output.
	 *
	 * @param value The current sensor value.
	 * @return The calculated output based on the PID algorithm.
	 *
	 *         The PID controller calculates the error between the setpoint and the
	 *         current sensor value.
	 *         It then applies proportional, integral, and derivative terms to the
	 *         error to determine the output.
	 *
	 *         If the 'continuous' flag is set to true, the error is adjusted to
	 *         handle wraparound situations
	 *         (e.g., when using absolute encoders).
	 *
	 *         The total error is clamped to prevent integral windup, and the output
	 *         is also clamped to the
	 *         specified minimum and maximum output values.
	 */
	public double updatePID(double value) {
		error = setPoint - value;
		if(thresholdOn){
			if(Math.abs(error) <= errorThreshold){
				return thresholdValue;
			}
		}

		if (continuous) {
			if (Math.abs(error) > (maxInput - minInput) / 2) {
				if (error > 0) {
					error = error - maxInput + minInput;
				} else {
					error = error + maxInput - minInput;
				}
			}
		}

		if ((error * PValue < maxOutput) && (error * PValue > minOutput)) {
			totalError += error;
		} else {
			totalError = 0;
		}

		result = (PValue * error + IValue * totalError + DValue * (error - prevError));
		prevError = error;
		result = clamp(result);
		result += gravity;
		return result;
	}

	/**
	 * Sets the proportional, integral, and derivative values for the PID
	 * controller.
	 *
	 * @param p The proportional gain. This value determines how much the controller
	 *          responds to the current error.
	 * @param i The integral gain. This value accumulates the error over time and
	 *          helps to eliminate steady-state errors.
	 * @param d The derivative gain. This value calculates the rate of change of the
	 *          error and helps to dampen oscillations.
	 *
	 *          The setPID() function updates the PValue, IValue, and DValue
	 *          variables with the provided values.
	 *          These values are used in the PID control algorithm to calculate the
	 *          output based on the current sensor value and setpoint.
	 */
	public void setPID(double p, double i, double d) {
		PValue = p;
		IValue = i;
		DValue = d;
	}


	public void setGravity(double g){
		gravity = g;
	}
	public double getGravity(){
		return gravity;
	}
	public void setThresholdOn(boolean  g){
		thresholdOn = g;
	}
	public boolean getThresholdOn(){
		return thresholdOn;
	}
	public void setErrorThreshold(double g){
		errorThreshold = g;
	}
	public double getErrorThreshold(){
		return errorThreshold;
	}
	public void setThresholdValue(double g){
		thresholdValue = g;
	}
	public double getThresholdValue(){
		return thresholdValue;
	}


	/**
	 * Retrieves the calculated output based on the PID algorithm.
	 *
	 * @return The calculated output, which is the result of the PID control
	 *         algorithm.
	 *         This value is clamped to the specified minimum and maximum output
	 *         values.
	 *
	 *         The getResult() function returns the result of the PID control
	 *         algorithm, which is
	 *         calculated by the updatePID() function. The result is clamped to
	 *         ensure it falls within
	 *         the specified minimum and maximum output values.
	 */
	public double getResult() {
		return result;
	}

	/**
	 * Sets the maximum output value for the PID controller.
	 *
	 * @param output The maximum output value. This value is used to clamp the
	 *               calculated PID output.
	 *               The output will be limited to this value if it exceeds it.
	 *
	 *               The setMaxOutput() function updates the maxOutput variable with
	 *               the provided value.
	 *               This value is used in the PID control algorithm to ensure that
	 *               the calculated output does not exceed the specified maximum
	 *               output value.
	 *               By limiting the output, the PID controller can prevent
	 *               overshooting or saturation issues in the system.
	 */
	public void setMaxOutput(double output) {
		maxOutput = output;
	}

	/**
	 * Sets the minimum output value for the PID controller.
	 *
	 * @param output The minimum output value. This value is used to clamp the
	 *               calculated PID output.
	 *               The output will be limited to this value if it falls below it.
	 *
	 *               The setMinOutput() function updates the minOutput variable with
	 *               the provided value.
	 *               This value is used in the PID control algorithm to ensure that
	 *               the calculated output does not fall below the specified minimum
	 *               output value.
	 *               By limiting the output, the PID controller can prevent
	 *               undershooting or saturation issues in the system.
	 */
	public void setMinOutput(double output) {
		minOutput = output;
	}

	/**
	 * Sets the minimum input value for the PID controller.
	 *
	 * @param input The minimum input value. This value is used to handle wraparound
	 *              situations
	 *              when using absolute encoders. If the current sensor value
	 *              exceeds the maximum
	 *              input value minus the minimum input value by more than half, the
	 *              error is adjusted
	 *              to account for the wraparound.
	 *
	 *              The setMinInput() function updates the minInput variable with
	 *              the provided value.
	 *              This value is used in the PID control algorithm to handle
	 *              wraparound situations when using
	 *              absolute encoders. By setting the minimum input value, the PID
	 *              controller can accurately
	 *              calculate the error and prevent integral windup in such cases.
	 */
	public void setMinInput(double input) {
		minInput = input;
	}

	/**
	 * Sets the maximum input value for the PID controller.
	 *
	 * @param input The maximum input value. This value is used to handle wraparound
	 *              situations
	 *              when using absolute encoders. If the current sensor value
	 *              exceeds the maximum
	 *              input value minus the minimum input value by more than half, the
	 *              error is adjusted
	 *              to account for the wraparound.
	 *
	 *              The setMaxInput() function updates the maxInput variable with
	 *              the provided value.
	 *              This value is used in the PID control algorithm to handle
	 *              wraparound situations when using
	 *              absolute encoders. By setting the minimum input value, the PID
	 *              controller can accurately
	 *              calculate the error and prevent integral windup in such cases.
	 */
	public void setMaxInput(double input) {
		maxInput = input;
	}

	/**
	 * Sets the target value of the PID controller
	 * 
	 * @param target
	 */
	public void setSetPoint(double target) {
		setPoint = target;
	}

	public void setContinuous(boolean value) {
		continuous = value;
	}

	/**
	 * Gives the current setpoint of the PID controller
	 * 
	 * @return The setpoint value of the PID controller
	 */
	public double getSetPoint() {
		return setPoint;
	}

	/**
	 * Clamps the value given between the maximum and minimum output values
	 * 
	 * @param input The value to clamp
	 * @return The clamped value
	 */
	public double clamp(double input) {
		if (input > maxOutput) {
			return maxOutput;
		}
		if (input < minOutput) {
			return minOutput;
		}
		return input;
	}

	@Override
	public String toString(){

		return "Target Position:" + getSetPoint() + "  Set Value: " + getResult() + "  Error: " + error + "  Threshold: " + thresholdOn + ", " + errorThreshold + ", " + thresholdValue;
	}
}