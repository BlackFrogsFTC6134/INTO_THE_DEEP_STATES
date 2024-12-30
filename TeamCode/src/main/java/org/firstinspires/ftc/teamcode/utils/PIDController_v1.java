package org.firstinspires.ftc.teamcode.utils;

// Refer to https://www.ctrlaltftc.com/practical-examples/ftc-motor-control

public class PIDController_v1 {

    // Factor for "proportional" control
    private double m_kp;

    // Factor for "integral" control
    private double m_ki;

    // Factor for "derivative" control
    private double m_kd;

    // The error range where "integral" control applies
    private double m_iZone = Double.POSITIVE_INFINITY;

    // The period (in seconds) of the loop that calls the controller
    private final double m_period;

    private double m_maximumIntegral = 1.0;

    private double m_minimumIntegral = -1.0;

    private double m_maximumInput;

    private double m_minimumInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    private boolean m_continuous;

    // The error at the time of the most recent call to calculate()
    private double m_positionError;
    private double m_velocityError;

    // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    private double m_prevError;

    // The sum of the errors for use in the integral calc
    private double m_totalError;

    // The error that is considered at setpoint.
    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;

    private double m_setpoint;
    private double m_measurement;

    private boolean m_haveMeasurement;
    private boolean m_haveSetpoint;

    private double Kp; // Factor for "proportional" control
    private double Ki; // Factor for "integral" control
    private double Kd; // Factor for "derivative" control
    private double previousError; // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    private double integral;

    /**
     * construct PID controller
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */

    public PIDController_v1(double Kp, double Ki, double Kd, double period) {
        m_kp = Kp;
        m_ki = Ki;
        m_kd = Kd;

        if (m_kp < 0.0) {
            throw new IllegalArgumentException("Kp must be a non-negative number!");
        }
        if (m_ki < 0.0) {
            throw new IllegalArgumentException("Ki must be a non-negative number!");
        }
        if (m_kd < 0.0) {
            throw new IllegalArgumentException("Kd must be a non-negative number!");
        }
        if (period <= 0.0) {
            throw new IllegalArgumentException("Controller period must be a positive number!");
        }

        m_period = period;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double kp) {
        m_kp = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double ki) {
        m_ki = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double kd) {
        m_kd = kd;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    /**
     * update the PID controller output
     * @param targetPosition where we would like to be, also called the reference
     * @param leftFrontPosition where we currently are, I.E. motor position
     * @param leftBackPosition where we currently are, I.E. motor position
     * @param leftBackPosition where we currently are, I.E. motor position
     * @param rightFrontPosition where we currently are, I.E. motor position
     * @param rightBackPosition where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        m_measurement = measurement;
        m_prevError = m_positionError;
        m_haveMeasurement = true;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_positionError = AngleWrap(m_setpoint - m_measurement);
        }

        m_velocityError = (m_positionError - m_prevError) / m_period;

        // If the absolute value of the position error is greater than IZone, reset the total error
        if (Math.abs(m_positionError) > m_iZone) {
            m_totalError = 0;
        } else if (m_ki != 0) {
            m_totalError =
                    MathUtil.clamp(
                            m_totalError + m_positionError * m_period,
                            m_minimumIntegral / m_ki,
                            m_maximumIntegral / m_ki);
        }

        return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
    }

    /**
     * Wraps radians to the range [-PI, PI].
     * The AngleWrap function you've provided is a common utility used in robotics and mathematics to normalize an angle to the range [-π, π] (or [-180°, 180°] in degrees). Here's an explanation of how it works:
     * Purpose: This function ensures that any angle, no matter how large or small, is "wrapped" to fit within the range of -π to π radians.
     * Process: If the input angle is greater than π, it subtracts 2π repeatedly until the angle falls within the desired range.
     * If the input angle is less than -π, it adds 2π repeatedly until the angle falls within the desired range.
     * Use cases:
             * This is particularly useful in robotics for:
             * Calculating the shortest path for rotation
             * Maintaining consistent angle representations
     * Preventing issues with angle overflow in calculations
     * @param radians = -PI to PI
     * @return = Wrapped angle to the range [-PI, PI]
     */
    public double AngleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));

     /* Alternate code (non optimized for large angles
             while (radians > Math.PI)
        {
            radians -= 2*Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2*Math.PI;

        }
        return radians;
      */
    }

    public double[] update (double targetPosition, double leftFrontPosition, double leftBackPosition, double rightFrontPosition, double rightBackPosition) {
        // PID logic and then return the output
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;

        // Calculate average current position for simplicity
        double currentPosition = (leftFrontPosition + leftBackPosition + rightFrontPosition + rightBackPosition) / 4;

        // Calculate error
        double error = targetPosition - currentPosition;

        // Integrate the errors as part of the integral term
        integral += error;

        // Derivative term is the difference between current error and previous error
        double derivative = error - previousError;

        // PID output calculation
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Update previous error for next iteration's derivative calculation
        previousError = error;

        // Assuming equal power distribution to all motors for simplicity; adjust as necessary for your application
        return new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        m_positionError = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_velocityError = 0;
        m_haveMeasurement = false;
    }
}