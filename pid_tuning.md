**Ziegler-Nichols** is a method of tuning PID controllers using the system's step response. Here's a step-by-step guide:

- Set the Integral (I) and Derivative (D) gains to zero. This leaves you with a simple Proportional (P) controller.

- Increase the Proportional gain (Kp) until the system oscillates. The system oscillates when it constantly overshoots and undershoots the desired value without settling. The Kp value at which this happens is known as the ultimate gain (Ku).

- Measure the oscillation period (Pu). This is the time it takes for the oscillations to complete one full cycle.

- Apply the Ziegler-Nichols tuning rules. These rules provide starting values for the PID gains based on Ku and Pu:

$
P controller: Kp = 0.5 * Ku
$

$
PI controller: Kp = 0.45 * Ku, Ki = 1.2 * Kp / Pu
$

$
PID controller: Kp = 0.6 * Ku, Ki = 2 * Kp / Pu, Kd = Kp * Pu / 8
$

Fine-tune the gains. The Ziegler-Nichols rules provide a good starting point, but you may need to adjust the gains to get the best performance for your specific system. This is often done through trial and error.

Remember, Ziegler-Nichols tuning assumes that your system can be approximated by a first-order process plus dead time (FOPDT) model. If your system doesn't meet this assumption, Ziegler-Nichols tuning may not give satisfactory results.