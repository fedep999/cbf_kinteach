#define S_FUNCTION_NAME  sfun_torque_cbf
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "torque_cbf/torque_cbf_controller.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <exception>
#include <memory>
#include <vector>

// Number of entries in the cbf_params input (Port 12)
#define N_CBF_PARAMS 25
#define N_JOINTS 7
#define N_TELEM 8
// Index of the first per-joint torque limit |tau_max,j| in cbf_params
#define IDX_TAU_MAX 17
// Index of the torque-rate budget for tau_cbf [Nm/s] (0 = disabled)
#define IDX_TAU_RATE 24

struct TorqueControllerInstance {
  torque_cbf::TorqueCBFController controller;
  double last_params[N_CBF_PARAMS] = {0.0};
  bool has_params = false;   // last_params holds a set of parameters already seen
  bool configured = false;   // controller holds a VALID configuration
  // Outputs of the last major time step, re-emitted in minor steps (zero-order hold)
  double last_tau[N_JOINTS] = {0.0};
  double last_telem[N_TELEM] = {0.0};
};

static void writeOutputs(SimStruct *S, const TorqueControllerInstance *inst)
{
  real_T *y_tau = ssGetOutputPortRealSignal(S, 0);
  real_T *y_telem = ssGetOutputPortRealSignal(S, 1);
  for (int i = 0; i < N_JOINTS; ++i) y_tau[i] = inst->last_tau[i];
  for (int i = 0; i < N_TELEM; ++i) y_telem[i] = inst->last_telem[i];
}

static void clearOutputs(TorqueControllerInstance *inst)
{
  for (int i = 0; i < N_JOINTS; ++i) inst->last_tau[i] = 0.0;
  for (int i = 0; i < N_TELEM; ++i) inst->last_telem[i] = 0.0;
}

#ifdef MATLAB_MEX_FILE
#define SFUN_WARN(S, msg) ssWarning(S, msg)
#else
#define SFUN_WARN(S, msg) ((void)0)
#endif

static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 0);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }

  // 13 Input Ports:
  // Port 0: q           (7x1 Joint Positions)
  // Port 1: dq          (7x1 Joint Velocities)
  // Port 2: x_ee        (3x1 End-Effector Cartesian Position)
  // Port 3: v_ee        (3x1 End-Effector Cartesian Linear Velocity)
  // Port 4: M           (7x7 Joint Mass Matrix)
  // Port 5: C           (7x7 Joint Coriolis Matrix)
  // Port 6: J           (6x7 Geometric Jacobian Matrix)
  // Port 7: dJ          (6x7 Jacobian Time Derivative Matrix)
  // Port 8: tau_nom     (7x1 Nominal Joint Motion Torque Vector: tau_task + tau_null)
  // Port 9: g           (7x1 Joint Gravity Vector)
  // Port 10: F_ext      (6x1 External Wrench)
  // Port 11: use_fext   (1x1 Boolean flag: 1 = compensate F_ext, 0 = ignore)
  // Port 12: cbf_params (25x1 Vector of CBF Hyperparameters & Plane Definition)
  //          [p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, nx, ny, nz, ox, oy, oz,
  //           latch_entry_velocity (0/1), zone_exit_hysteresis [m], latch_velocity_filter_alpha (0,1],
  //           tau_max_1 ... tau_max_7 [Nm],
  //           tau_rate_limit [Nm/s]: rate budget of tau_cbf (0 = off; e.g. 900 with the Franka 1000 Nm/s
  //           limit, leaving headroom for tau_null). Enables the rate bound and the pushback-aware cap.]
  //          tau_max_j > 0 are the per-joint limits, applied symmetrically to the TOTAL torque
  //          g + tau_nom + tau_cbf (tau_limit is kept only as a fallback and is unused here).
  if (!ssSetNumInputPorts(S, 13)) return;

  // Port 0: q (7x1)
  ssSetInputPortWidth(S, 0, 7);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortRequiredContiguous(S, 0, 1);

  // Port 1: dq (7x1)
  ssSetInputPortWidth(S, 1, 7);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortRequiredContiguous(S, 1, 1);

  // Port 2: x_ee (3x1)
  ssSetInputPortWidth(S, 2, 3);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortRequiredContiguous(S, 2, 1);

  // Port 3: v_ee (3x1)
  ssSetInputPortWidth(S, 3, 3);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortRequiredContiguous(S, 3, 1);

  // Port 4: M (7x7)
  ssSetInputPortMatrixDimensions(S, 4, 7, 7);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortRequiredContiguous(S, 4, 1);

  // Port 5: C (7x7)
  ssSetInputPortMatrixDimensions(S, 5, 7, 7);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetInputPortRequiredContiguous(S, 5, 1);

  // Port 6: J (6x7)
  ssSetInputPortMatrixDimensions(S, 6, 6, 7);
  ssSetInputPortDirectFeedThrough(S, 6, 1);
  ssSetInputPortRequiredContiguous(S, 6, 1);

  // Port 7: dJ (6x7)
  ssSetInputPortMatrixDimensions(S, 7, 6, 7);
  ssSetInputPortDirectFeedThrough(S, 7, 1);
  ssSetInputPortRequiredContiguous(S, 7, 1);

  // Port 8: tau_nom (7x1)
  ssSetInputPortWidth(S, 8, 7);
  ssSetInputPortDirectFeedThrough(S, 8, 1);
  ssSetInputPortRequiredContiguous(S, 8, 1);

  // Port 9: g (7x1)
  ssSetInputPortWidth(S, 9, 7);
  ssSetInputPortDirectFeedThrough(S, 9, 1);
  ssSetInputPortRequiredContiguous(S, 9, 1);

  // Port 10: F_ext (6x1 wrench)
  ssSetInputPortWidth(S, 10, 6);
  ssSetInputPortDirectFeedThrough(S, 10, 1);
  ssSetInputPortRequiredContiguous(S, 10, 1);

  // Port 11: use_fext (1x1 flag)
  ssSetInputPortWidth(S, 11, 1);
  ssSetInputPortDirectFeedThrough(S, 11, 1);
  ssSetInputPortRequiredContiguous(S, 11, 1);

  // Port 12: cbf_params (25x1)
  ssSetInputPortWidth(S, 12, N_CBF_PARAMS);
  ssSetInputPortDirectFeedThrough(S, 12, 1);
  ssSetInputPortRequiredContiguous(S, 12, 1);

  // 2 Output Ports:
  // Port 0: tau_cbf   (7x1 Vector of CBF Joint Torques)
  // Port 1: telemetry (8x1 Vector: [is_active, zeta, dzeta, v_min, barrier_B, f_n_scalar, slack_delta, qp_cost])
  if (!ssSetNumOutputPorts(S, 2)) return;

  ssSetOutputPortWidth(S, 0, N_JOINTS);
  ssSetOutputPortWidth(S, 1, N_TELEM);

  ssSetNumSampleTimes(S, 1);
  ssSetNumPWork(S, 1);

  ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  auto *inst = new TorqueControllerInstance();
  ssSetPWorkValue(S, 0, inst);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  auto *inst = static_cast<TorqueControllerInstance*>(ssGetPWorkValue(S, 0));
  if (!inst) return;

  // With an inherited CONTINUOUS rate (e.g. ode4 + Simscape plant) mdlOutputs is also called
  // in minor (intermediate) solver steps. The controller is stateful (entry latch, latch filter)
  // and models a sampled controller, so it is evaluated only in major steps; minor steps
  // re-emit the last outputs (zero-order hold, as on the real 1 kHz robot).
  if (!ssIsMajorTimeStep(S)) {
    writeOutputs(S, inst);
    return;
  }

  // Read Inputs
  const real_T *u_q       = ssGetInputPortRealSignal(S, 0);
  const real_T *u_dq      = ssGetInputPortRealSignal(S, 1);
  const real_T *u_x       = ssGetInputPortRealSignal(S, 2);
  const real_T *u_v       = ssGetInputPortRealSignal(S, 3);
  const real_T *u_M       = ssGetInputPortRealSignal(S, 4);
  const real_T *u_C       = ssGetInputPortRealSignal(S, 5);
  const real_T *u_J       = ssGetInputPortRealSignal(S, 6);
  const real_T *u_dJ      = ssGetInputPortRealSignal(S, 7);
  const real_T *u_taunom  = ssGetInputPortRealSignal(S, 8);
  const real_T *u_g       = ssGetInputPortRealSignal(S, 9);
  const real_T *u_Fext    = ssGetInputPortRealSignal(S, 10);
  const real_T use_fext   = *ssGetInputPortRealSignal(S, 11);
  const real_T *u_params  = ssGetInputPortRealSignal(S, 12);

  // Map to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(u_q);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(u_dq);
  Eigen::Vector3d x_ee(u_x[0], u_x[1], u_x[2]);
  Eigen::Vector3d dx_ee(u_v[0], u_v[1], u_v[2]);
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(u_M);
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> C(u_C);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(u_J);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> dJ(u_dJ);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_nom(u_taunom);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> g(u_g);

  // F_ext (6x1)
  Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
  if (use_fext > 0.5) {
    for (int i = 0; i < 6; ++i) F_ext(i) = u_Fext[i];
  }

  // (Re)configure ONLY when the parameter vector changes, validating first so an invalid
  // parameter set can never throw out of this exception-free S-function.
  // On invalid parameters the previous valid configuration is kept.
  bool params_changed = !inst->has_params;
  for (int i = 0; i < N_CBF_PARAMS && !params_changed; ++i) {
    params_changed = (u_params[i] != inst->last_params[i]);
  }
  if (params_changed) {
    for (int i = 0; i < N_CBF_PARAMS; ++i) inst->last_params[i] = u_params[i];
    inst->has_params = true;

    bool finite = true;
    for (int i = 0; i < N_CBF_PARAMS; ++i) finite = finite && std::isfinite(u_params[i]);

    torque_cbf::TorqueCBFConfig config;
    config.p_ths                       = u_params[0];
    config.v_ths                       = u_params[1];
    config.v_0                         = u_params[2];
    config.gamma                       = u_params[3];
    config.w_lin                       = u_params[4];
    config.w_slack                     = u_params[5];
    config.default_tau_limit           = u_params[6];
    config.dt                          = u_params[7];
    config.latch_entry_velocity        = (u_params[14] > 0.5);
    config.zone_exit_hysteresis        = u_params[15];
    config.latch_velocity_filter_alpha = u_params[16];
    config.profile                     = cbf::ProfileType::SMOOTHSTEP;
    config.compensate_external_force   = (use_fext > 0.5);
    config.enforce_passivity           = true;
    config.tau_rate_limit              = u_params[IDX_TAU_RATE];
    if (!(config.tau_rate_limit >= 0.0)) finite = false;   // reject negative rate budget

    // Per-joint limits on the TOTAL torque g + tau_nom + tau_cbf (symmetric)
    config.tau_max.resize(N_JOINTS);
    for (int j = 0; j < N_JOINTS; ++j) {
      config.tau_max(j) = u_params[IDX_TAU_MAX + j];
      if (!(config.tau_max(j) > 0.0)) finite = false;   // reject non-positive limits
    }
    config.tau_min = -config.tau_max;
    config.enforce_total_torque_limits = true;

    cbf::Hyperplane plane;
    plane.normal = Eigen::Vector3d(u_params[8], u_params[9], u_params[10]);
    plane.offset = Eigen::Vector3d(u_params[11], u_params[12], u_params[13]);

    if (!finite) {
      SFUN_WARN(S, "sfun_torque_cbf: non-finite cbf_params or non-positive torque limits ignored, keeping previous configuration.");
    } else {
      try {
        // Validate on a scratch copy first, so a bad set never leaves the controller half-updated
        torque_cbf::TorqueCBFController probe(config, plane);
        (void)probe;
        inst->controller.setConfig(config);
        inst->controller.setHyperplane(plane);
        inst->configured = true;
      } catch (const std::exception &) {
        SFUN_WARN(S, "sfun_torque_cbf: invalid cbf_params ignored, keeping previous configuration.");
      }
    }
  }

  // No valid configuration yet: no CBF torque
  if (!inst->configured) {
    clearOutputs(inst);
    writeOutputs(S, inst);
    return;
  }

  // F_ext compensation flag is a separate input; not part of the change-detected config
  {
    torque_cbf::TorqueCBFConfig cfg = inst->controller.getConfig();
    const bool comp = (use_fext > 0.5);
    if (cfg.compensate_external_force != comp) {
      cfg.compensate_external_force = comp;
      inst->controller.setConfig(cfg);   // already validated: cannot throw
    }
  }

  // Populate RobotState struct
  torque_cbf::RobotState state;
  state.q     = q;
  state.dq    = dq;
  state.x     = x_ee;
  state.dx    = dx_ee;
  state.M     = M;
  state.C     = C;
  state.g     = g;
  state.J     = J;
  state.dJ    = dJ;
  state.F_ext = F_ext;

  torque_cbf::TorqueCBFResult res;
  try {
    inst->controller.computeControlTorque(state, tau_nom, &res);
  } catch (const std::exception &) {
    // Solver failure: command NO CBF torque this step (a stale torque could push the robot)
    SFUN_WARN(S, "sfun_torque_cbf: CBF step failed, tau_cbf set to zero.");
    inst->controller.resetRateState();   // next step plans from zero force, as commanded now
    clearOutputs(inst);
    writeOutputs(S, inst);
    return;
  }

  // Output Port 0: tau_cbf (7x1)
  for (int i = 0; i < N_JOINTS; ++i) {
    inst->last_tau[i] = res.tau_cbf(i);
  }

  // Output Port 1: Telemetry Vector (8x1)
  inst->last_telem[0] = res.is_active ? 1.0 : 0.0;
  inst->last_telem[1] = res.zeta;
  inst->last_telem[2] = res.dzeta;
  inst->last_telem[3] = res.v_min;
  inst->last_telem[4] = res.barrier_B;
  inst->last_telem[5] = res.f_n_scalar;
  inst->last_telem[6] = res.slack_delta;
  inst->last_telem[7] = res.qp_cost;
  writeOutputs(S, inst);
}

static void mdlTerminate(SimStruct *S)
{
  auto *inst = static_cast<TorqueControllerInstance*>(ssGetPWorkValue(S, 0));
  if (inst) {
    delete inst;
    ssSetPWorkValue(S, 0, nullptr);
  }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
