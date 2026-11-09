#define S_FUNCTION_NAME  sfun_damping_cbf
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "damping_cbf/damping_cbf_controller.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <exception>
#include <memory>

// Number of entries in the cbf_params input (Port 6)
#define N_CBF_PARAMS 16

struct DampingInstance {
  damping_cbf::DampingCBFController controller;
  double last_params[N_CBF_PARAMS] = {0.0};
  bool has_params = false;   // last_params holds a set of parameters already seen
  bool configured = false;   // controller holds a VALID configuration
};

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

  // 7 Input Ports:
  // Port 0: zeta        (1x1 Distance along normal to hyperplane x_c [m])
  // Port 1: dzeta       (1x1 Velocity along normal v_c [m/s], true ground velocity)
  // Port 2: rho         (1x1 Effective Cartesian inertia m_c [kg])
  // Port 3: F_nominal   (6x1 Nominal Cartesian impedance wrench [N, Nm])
  // Port 4: F_ext       (6x1 External wrench [N, Nm])
  // Port 5: use_fext    (1x1 Boolean flag: 1 = compensate F_ext in drift, 0 = ignore)
  // Port 6: cbf_params  (16x1 Vector of CBF Hyperparameters & Plane Definition)
  //         [p_ths, v_ths, v_0, gamma, d_max, w_lin, w_quad, nx, ny, nz, ox, oy, oz,
  //          latch_entry_velocity (0/1), zone_exit_hysteresis [m], latch_velocity_filter_alpha (0,1]]
  if (!ssSetNumInputPorts(S, 7)) return;

  // Port 0: zeta (1x1)
  ssSetInputPortWidth(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortRequiredContiguous(S, 0, 1);

  // Port 1: dzeta (1x1)
  ssSetInputPortWidth(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortRequiredContiguous(S, 1, 1);

  // Port 2: rho (1x1)
  ssSetInputPortWidth(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortRequiredContiguous(S, 2, 1);

  // Port 3: F_nominal (6x1)
  ssSetInputPortWidth(S, 3, 6);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortRequiredContiguous(S, 3, 1);

  // Port 4: F_ext (6x1)
  ssSetInputPortWidth(S, 4, 6);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortRequiredContiguous(S, 4, 1);

  // Port 5: use_fext (1x1)
  ssSetInputPortWidth(S, 5, 1);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetInputPortRequiredContiguous(S, 5, 1);

  // Port 6: cbf_params (16x1)
  ssSetInputPortWidth(S, 6, N_CBF_PARAMS);
  ssSetInputPortDirectFeedThrough(S, 6, 1);
  ssSetInputPortRequiredContiguous(S, 6, 1);

  // 2 Output Ports:
  // Port 0: d_cbf      (1x1 Optimal Numerical QP Damping Coefficient in [0, d_max])
  // Port 1: telemetry  (8x1 Vector: [is_active, zeta, dzeta, v_min, barrier_B, Lf_B, Lg_B, slack_eps])
  if (!ssSetNumOutputPorts(S, 2)) return;

  ssSetOutputPortWidth(S, 0, 1);
  ssSetOutputPortWidth(S, 1, 8);

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
  auto *inst = new DampingInstance();
  ssSetPWorkValue(S, 0, inst);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  auto *inst = static_cast<DampingInstance*>(ssGetPWorkValue(S, 0));
  if (!inst) return;

  // Read Inputs
  const real_T zeta     = *ssGetInputPortRealSignal(S, 0);
  const real_T dzeta    = *ssGetInputPortRealSignal(S, 1);
  const real_T rho      = *ssGetInputPortRealSignal(S, 2);
  const real_T *u_Fnom  = ssGetInputPortRealSignal(S, 3);
  const real_T *u_Fext  = ssGetInputPortRealSignal(S, 4);
  const real_T use_fext = *ssGetInputPortRealSignal(S, 5);
  const real_T *params  = ssGetInputPortRealSignal(S, 6);

  // (Re)configure ONLY when the parameter vector changes: calling setConfig() every step is
  // harmless for the controller now, but skipping it avoids needless work, and validating here
  // means an invalid parameter set can never throw out of this exception-free S-function.
  // On invalid parameters the previous valid configuration is kept.
  bool params_changed = !inst->has_params;
  for (int i = 0; i < N_CBF_PARAMS && !params_changed; ++i) {
    params_changed = (params[i] != inst->last_params[i]);
  }
  if (params_changed) {
    for (int i = 0; i < N_CBF_PARAMS; ++i) inst->last_params[i] = params[i];
    inst->has_params = true;

    bool finite = true;
    for (int i = 0; i < N_CBF_PARAMS; ++i) finite = finite && std::isfinite(params[i]);

    damping_cbf::ControllerConfig config;
    config.p_ths                       = params[0];
    config.v_ths                       = params[1];
    config.v_0                         = params[2];
    config.gamma                       = params[3];
    config.d_max                       = params[4];
    config.slack_linear_penalty        = params[5];
    config.slack_quadratic_penalty     = params[6];
    config.latch_entry_velocity        = (params[13] > 0.5);
    config.zone_exit_hysteresis        = params[14];
    config.latch_velocity_filter_alpha = params[15];
    config.profile                     = cbf::ProfileType::SMOOTHSTEP;

    cbf::Hyperplane plane;
    plane.normal = Eigen::Vector3d(params[7], params[8], params[9]);
    plane.offset = Eigen::Vector3d(params[10], params[11], params[12]);

    if (!finite) {
      SFUN_WARN(S, "sfun_damping_cbf: non-finite cbf_params ignored, keeping previous configuration.");
    } else {
      try {
        // Validate on a scratch copy first, so a bad set never leaves the controller half-updated
        damping_cbf::DampingCBFController probe(config, plane);
        (void)probe;
        inst->controller.setConfig(config);
        inst->controller.setHyperplane(plane);
        inst->configured = true;
      } catch (const std::exception &) {
        SFUN_WARN(S, "sfun_damping_cbf: invalid cbf_params ignored, keeping previous configuration.");
      }
    }
  }

  // No valid configuration yet: hold outputs (initially zero = no CBF damping)
  if (!inst->configured) return;

  // F_ext compensation flag is a separate input; not part of the change-detected config
  {
    damping_cbf::ControllerConfig cfg = inst->controller.getConfig();
    const bool comp = (use_fext > 0.5);
    if (cfg.compensate_external_force != comp) {
      cfg.compensate_external_force = comp;
      inst->controller.setConfig(cfg);   // already validated: cannot throw
    }
  }

  try {
    // Compute normal drift acceleration a_drift = (n6^T * (F_nom + (use_fext ? F_ext : 0))) / rho
    Eigen::Matrix<double, 6, 1> F_nom;
    for (int i = 0; i < 6; ++i) F_nom(i) = u_Fnom[i];

    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
    if (use_fext > 0.5) {
      for (int i = 0; i < 6; ++i) F_ext(i) = u_Fext[i];
    }

    const Eigen::Matrix<double, 6, 1> F_drift = F_nom + F_ext;
    Eigen::Matrix<double, 6, 1> n6 = Eigen::Matrix<double, 6, 1>::Zero();
    n6.head<3>() = inst->controller.getHyperplane().normal;   // already normalized

    const double a_drift = (rho > 1e-9) ? (n6.dot(F_drift) / rho) : 0.0;

    // Solve the Damping CBF Quadratic Program numerically via Goldfarb-Idnani (quadprog)
    damping_cbf::CBFStepResult res = inst->controller.solveDampingQP(zeta, dzeta, rho, a_drift);

    // Output 0: d_cbf
    real_T *y_damp = ssGetOutputPortRealSignal(S, 0);
    y_damp[0] = res.d_cbf;

    // Output 1: Telemetry
    real_T *y_telem = ssGetOutputPortRealSignal(S, 1);
    y_telem[0] = res.is_active ? 1.0 : 0.0;
    y_telem[1] = res.zeta;
    y_telem[2] = res.dzeta;
    y_telem[3] = res.v_min;
    y_telem[4] = res.barrier_B;
    y_telem[5] = res.Lf_B;
    y_telem[6] = res.Lg_B;
    y_telem[7] = res.slack_eps;
  } catch (const std::exception &) {
    // Solver failure: hold the previous outputs for this step (damping d >= 0 is always passive)
    SFUN_WARN(S, "sfun_damping_cbf: CBF step failed, holding previous output.");
  }
}

static void mdlTerminate(SimStruct *S)
{
  auto *inst = static_cast<DampingInstance*>(ssGetPWorkValue(S, 0));
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
