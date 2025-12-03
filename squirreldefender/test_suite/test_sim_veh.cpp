

/********************************************************************************************
 * Drone Velocity Plant Model (Identified via ARX(1) Least Squares)
 *
 * Discrete-time model (21 Hz ≈ dt = 0.0476 s):
 *
 *      v[k+1] = a * v[k] + b * u[k]
 *
 * Identified parameters (4 datasets):
 *      a ≈ 0.94 – 0.97
 *      b ≈ 0.015 – 0.037
 *
 * Recommended averaged model for simulation:
 *
 *      a = 0.954f;
 *      b = 0.030f;
 *
 * Interpretation:
 *  - |a| < 1  → stable first-order system
 *  - a close to 1 means the velocity has significant inertia (strong memory)
 *  - b determines how strongly the control input u influences the velocity
 *
 * Steady-state gain:
 *
 *      v_ss = (b / (1 - a)) * u  ≈ 0.65 * u   (m/s per unit command)
 *
 * With dt = ~0.0476 s, the equivalent continuous-time system is:
 *
 *      dv/dt = -α * v + β * u
 *
 * where:
 *
 *      α = -ln(a) / dt
 *      β = b * α / (1 - a)
 *
 * Using the averaged parameters:
 *
 *      α ≈ 0.99  (1/s)
 *      β ≈ 0.65  (m/s per unit command per second)
 *
 * Time constant:
 *
 *      τ = 1 / α ≈ 1.0 s
 *
 * Meaning the velocity reaches:
 *      ~63% of its response in ≈1.0 s
 *      ~95% of its response in ≈3.0 s
 *
 * -------------------------------------------------------------------------------------------
 * Discrete-time plant step (recommended for sim):
 *
 *      float v_next = a * v + b * u;
 *
 * Continuous-time plant ODE form:
 *
 *      dv/dt = -α * v + β * u;
 *
 * -------------------------------------------------------------------------------------------
 * These values let the controller/simulator accurately mimic the real drone’s
 * forward velocity dynamics for MPC design, sim-follow, and feed-forward shaping.
 ********************************************************************************************/

// dt ~ 1/21 s
// Discrete veh motion model
float a = 0.954f; // average of your runs
float b = 0.030f;

float sim_veh_model_discrete(float v, float u)
{
    return a * v + b * u;
}

// Continuous time model
const float alpha = 0.99f; // 1/s
const float beta = 0.65f;  // (m/s) / (u * s)

float sim_veh_model_continous(float v, float u)
{
    return -alpha * v + beta * u;
}
