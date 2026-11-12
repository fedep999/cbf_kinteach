#define S_FUNCTION_NAME  sfun_joint_torque_cbf
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "jtorque_cbf/joint_torque_cbf_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <exception>

// Same 13 inputs as sfun_torque_cbf (drop-in replacement in the torque model), longer parameter vector.
#define N_CBF_PARAMS 31
#define N_JOINTS 7
#define N_TELEM 8
#define N_GUAR 6
// Indices in cbf_params
#define IDX_TAU_MAX 17          // 17..23: per-joint torque limits |tau_max,j| [Nm]
#define IDX_TAU_RATE 24         // robot torque-rate limit on tau_nom + tau_cbf [Nm/s] (0 = off)
#define IDX_MAX_LAT_FORCE 25    // bound on each lateral force component [N]
#define IDX_MAX_MOMENT 26       // bound on each moment component [Nm]
#define IDX_W_LATERAL 27        // cost weight of the lateral force (normal = 1)
#define IDX_W_MOMENT 28         // cost weight of the moment
#define IDX_W_NULL 29           // cost weight of the nullspace torque
#define IDX_GUAR_FACTOR 30      // guarantee slack penalty relative to the CBF slack (>= 1)

struct JointTorqueInstance {
  jtorque_cbf::JointTorqueCBFController controller;
  double last_params[N_CBF_PARAMS] = {0.0};
  bool has_params = false;   // last_params holds a parameter set already seen
  bool configured = false;   // controller holds a VALID configuration
  // Outputs of the last major time step, re-emitted in minor steps (zero-order hold)
  double last_tau[N_JOINTS] = {0.0};
  double last_telem[N_TELEM] = {0.0};
  double last_guar[N_GUAR] = {0.0};
};

static void writeOutputs(SimStruct *S, const JointTorqueInstance *inst)
{
  real_T *y_tau = ssGetOutputPortRealSignal(S, 0);
  real_T *y_telem = ssGetOutputPortRealSignal(S, 1);
  real_T *y_guar = ssGetOutputPortRealSignal(S, 2);
  for (int i = 0; i < N_JOINTS; ++i) y_tau[i] = inst->last_tau[i];
  for (int i = 0; i < N_TELEM; ++i) y_telem[i] = inst->last_telem[i];
  for (int i = 0; i < N_GUAR; ++i) y_guar[i] = inst->last_guar[i];
}

static void clearOutputs(JointTorqueInstance *inst)
{
  for (int i = 0; i < N_JOINTS; ++i) inst->last_tau[i] = 0.0;
  for (int i = 0; i < N_TELEM; ++i) inst->last_telem[i] = 0.0;
  for (int i = 0; i < N_GUAR; ++i) inst->last_guar[i] = 0.0;
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

  // 13 Input Ports (identical to sfun_torque_cbf):
  // Port 0: q           (7x1 Joint positions)
  // Port 1: dq          (7x1 Joint velocities)
  // Port 2: x_ee        (3x1 End-effector position)
  // Port 3: v_ee        (3x1 End-effector linear velocity)
  // Port 4: M           (7x7 Joint inertia matrix)
  // Port 5: C           (7x7 Joint Coriolis matrix)
  // Port 6: J           (6x7 Geometric Jacobian [lin; ang])
  // Port 7: dJ          (6x7 TRUE time derivative of J)
  // Port 8: tau_nom     (7x1 Other commanded torques: tau_null + tau_stiff; NOT gravity)
  // Port 9: g           (7x1 Gravity torque, used only for the total torque limits)
  // Port 10: F_ext      (6x1 External wrench at the Jacobian point)
  // Port 11: use_fext   (1x1 1 = include F_ext in the drift)
  // Port 12: cbf_params (31x1)
  //          [p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, nx, ny, nz, ox, oy, oz,
  //           latch_entry_velocity (0/1), zone_exit_hysteresis [m], latch_velocity_filter_alpha (0,1],
  //           tau_max_1 ... tau_max_7 [Nm],
  //           tau_rate_limit [Nm/s]   robot rate limit on tau_nom + tau_cbf (999 = Simulink Rate Limiter; 0 = off),
  //           max_lateral_force [N], max_moment [Nm]   (0, 0 = pure normal force as torque_cbf),
  //           w_lateral, w_moment, w_null              (cost weights, normal force = 1),
  //           guarantee_slack_factor                   (>= 1, e.g. 1e3)]
  if (!ssSetNumInputPorts(S, 13)) return;

  const int widths[13] = {7, 7, 3, 3, -1, -1, -1, -1, 7, 7, 6, 1, N_CBF_PARAMS};
  for (int p = 0; p < 13; ++p) {
    if (p == 4 || p == 5) {
      ssSetInputPortMatrixDimensions(S, p, 7, 7);
    } else if (p == 6 || p == 7) {
      ssSetInputPortMatrixDimensions(S, p, 6, 7);
    } else {
      ssSetInputPortWidth(S, p, widths[p]);
    }
    ssSetInputPortDirectFeedThrough(S, p, 1);
    ssSetInputPortRequiredContiguous(S, p, 1);
  }

  // 3 Output Ports:
  // Port 0: tau_cbf    (7x1 CBF joint torque)
  // Port 1: telemetry  (8x1, same layout as sfun_torque_cbf:
  //                     [is_active, zeta, dzeta, v_min, barrier_B, f_n (normal-equivalent force), slack_delta, qp_cost])
  // Port 2: guarantees (6x1: [sigma_normal [N], sigma_power [W], sigma_lateral [N/Nm],
  //                           rate_limited (0/1), max lateral force [N], CBF power tau_cbf'*dq [W]])
  //                     sigma_* > 0 means the actuator physically could not satisfy that guarantee this step
  if (!ssSetNumOutputPorts(S, 3)) return;
  ssSetOutputPortWidth(S, 0, N_JOINTS);
  ssSetOutputPortWidth(S, 1, N_TELEM);
  ssSetOutputPortWidth(S, 2, N_GUAR);

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
  auto *inst = new JointTorqueInstance();
  ssSetPWorkValue(S, 0, inst);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  (void)tid;
  auto *inst = static_cast<JointTorqueInstance*>(ssGetPWorkValue(S, 0));
  if (!inst) return;

  // Stateful sampled controller (entry latch, rate limit): evaluate only in major steps,
  // re-emit the last outputs in minor steps (zero-order hold, as on the real 1 kHz robot).
  if (!ssIsMajorTimeStep(S)) {
    writeOutputs(S, inst);
    return;
  }

  const real_T *u_q      = ssGetInputPortRealSignal(S, 0);
  const real_T *u_dq     = ssGetInputPortRealSignal(S, 1);
  const real_T *u_x      = ssGetInputPortRealSignal(S, 2);
  const real_T *u_v      = ssGetInputPortRealSignal(S, 3);
  const real_T *u_M      = ssGetInputPortRealSignal(S, 4);
  const real_T *u_C      = ssGetInputPortRealSignal(S, 5);
  const real_T *u_J      = ssGetInputPortRealSignal(S, 6);
  const real_T *u_dJ     = ssGetInputPortRealSignal(S, 7);
  const real_T *u_taunom = ssGetInputPortRealSignal(S, 8);
  const real_T *u_g      = ssGetInputPortRealSignal(S, 9);
  const real_T *u_Fext   = ssGetInputPortRealSignal(S, 10);
  const real_T use_fext  = *ssGetInputPortRealSignal(S, 11);
  const real_T *u_params = ssGetInputPortRealSignal(S, 12);

  // (Re)configure only when the parameter vector changes; validate first so an invalid set can
  // never throw out of this exception-free S-function (the previous valid configuration is kept).
  bool params_changed = !inst->has_params;
  for (int i = 0; i < N_CBF_PARAMS && !params_changed; ++i) {
    params_changed = (u_params[i] != inst->last_params[i]);
  }
  if (params_changed) {
    for (int i = 0; i < N_CBF_PARAMS; ++i) inst->last_params[i] = u_params[i];
    inst->has_params = true;

    bool valid = true;
    for (int i = 0; i < N_CBF_PARAMS; ++i) valid = valid && std::isfinite(u_params[i]);

    jtorque_cbf::JointTorqueCBFConfig config;
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
    config.tau_max.resize(N_JOINTS);
    for (int j = 0; j < N_JOINTS; ++j) {
      config.tau_max(j) = u_params[IDX_TAU_MAX + j];
      if (!(config.tau_max(j) > 0.0)) valid = false;   // reject non-positive limits
    }
    config.tau_min = -config.tau_max;
    config.enforce_total_torque_limits = true;
    config.tau_rate_limit         = u_params[IDX_TAU_RATE];
    config.max_lateral_force      = u_params[IDX_MAX_LAT_FORCE];
    config.max_moment             = u_params[IDX_MAX_MOMENT];
    config.w_lateral              = u_params[IDX_W_LATERAL];
    config.w_moment               = u_params[IDX_W_MOMENT];
    config.w_null                 = u_params[IDX_W_NULL];
    config.guarantee_slack_factor = u_params[IDX_GUAR_FACTOR];

    cbf::Hyperplane plane;
    plane.normal = Eigen::Vector3d(u_params[8], u_params[9], u_params[10]);
    plane.offset = Eigen::Vector3d(u_params[11], u_params[12], u_params[13]);

    if (!valid) {
      SFUN_WARN(S, "sfun_joint_torque_cbf: non-finite cbf_params or non-positive torque limits ignored, keeping previous configuration.");
    } else {
      try {
        // Validate on a scratch copy first, so a bad set never leaves the controller half-updated
        jtorque_cbf::JointTorqueCBFController probe(config, plane);
        (void)probe;
        inst->controller.setConfig(config);
        inst->controller.setHyperplane(plane);
        inst->configured = true;
      } catch (const std::exception &) {
        SFUN_WARN(S, "sfun_joint_torque_cbf: invalid cbf_params ignored, keeping previous configuration.");
      }
    }
  }

  if (!inst->configured) {
    clearOutputs(inst);
    writeOutputs(S, inst);
    return;
  }

  // F_ext compensation flag is a separate input, not part of the change-detected parameters
  {
    jtorque_cbf::JointTorqueCBFConfig cfg = inst->controller.getConfig();
    const bool comp = (use_fext > 0.5);
    if (cfg.compensate_external_force != comp) {
      cfg.compensate_external_force = comp;
      inst->controller.setConfig(cfg);   // already validated: cannot throw
    }
  }

  jtorque_cbf::RobotState state;
  state.q  = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(u_q);
  state.dq = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(u_dq);
  state.x  = Eigen::Vector3d(u_x[0], u_x[1], u_x[2]);
  state.dx = Eigen::Vector3d(u_v[0], u_v[1], u_v[2]);
  state.M  = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(u_M);
  state.C  = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(u_C);
  state.J  = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(u_J);
  state.dJ = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(u_dJ);
  state.g  = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(u_g);
  state.F_ext.setZero();
  if (use_fext > 0.5) {
    for (int i = 0; i < 6; ++i) state.F_ext(i) = u_Fext[i];
  }
  const Eigen::VectorXd tau_nom = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(u_taunom);

  jtorque_cbf::JointTorqueCBFResult res;
  try {
    inst->controller.computeControlTorque(state, tau_nom, &res);
  } catch (const std::exception &) {
    // Solver failure: command no CBF torque this step (a stale torque could push the robot)
    SFUN_WARN(S, "sfun_joint_torque_cbf: CBF step failed, tau_cbf set to zero.");
    inst->controller.resetRateState();
    clearOutputs(inst);
    writeOutputs(S, inst);
    return;
  }

  for (int i = 0; i < N_JOINTS; ++i) inst->last_tau[i] = res.tau_cbf(i);

  inst->last_telem[0] = res.is_active ? 1.0 : 0.0;
  inst->last_telem[1] = res.zeta;
  inst->last_telem[2] = res.dzeta;
  inst->last_telem[3] = res.v_min;
  inst->last_telem[4] = res.barrier_B;
  inst->last_telem[5] = res.f_n_scalar;
  inst->last_telem[6] = res.slack_delta;
  inst->last_telem[7] = res.qp_cost;

  inst->last_guar[0] = res.sigma_normal;
  inst->last_guar[1] = res.sigma_power;
  inst->last_guar[2] = res.sigma_lateral;
  inst->last_guar[3] = res.rate_limited ? 1.0 : 0.0;
  inst->last_guar[4] = res.F_lateral;
  inst->last_guar[5] = res.power;
  writeOutputs(S, inst);
}

static void mdlTerminate(SimStruct *S)
{
  auto *inst = static_cast<JointTorqueInstance*>(ssGetPWorkValue(S, 0));
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
