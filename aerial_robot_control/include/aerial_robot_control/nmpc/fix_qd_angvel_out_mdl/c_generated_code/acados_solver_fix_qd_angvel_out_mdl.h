/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_fix_qd_angvel_out_mdl_H_
#define ACADOS_SOLVER_fix_qd_angvel_out_mdl_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define FIX_QD_ANGVEL_OUT_MDL_NX     10
#define FIX_QD_ANGVEL_OUT_MDL_NZ     0
#define FIX_QD_ANGVEL_OUT_MDL_NU     4
#define FIX_QD_ANGVEL_OUT_MDL_NP     4
#define FIX_QD_ANGVEL_OUT_MDL_NP_GLOBAL     0
#define FIX_QD_ANGVEL_OUT_MDL_NBX    3
#define FIX_QD_ANGVEL_OUT_MDL_NBX0   10
#define FIX_QD_ANGVEL_OUT_MDL_NBU    4
#define FIX_QD_ANGVEL_OUT_MDL_NSBX   0
#define FIX_QD_ANGVEL_OUT_MDL_NSBU   0
#define FIX_QD_ANGVEL_OUT_MDL_NSH    0
#define FIX_QD_ANGVEL_OUT_MDL_NSH0   0
#define FIX_QD_ANGVEL_OUT_MDL_NSG    0
#define FIX_QD_ANGVEL_OUT_MDL_NSPHI  0
#define FIX_QD_ANGVEL_OUT_MDL_NSHN   0
#define FIX_QD_ANGVEL_OUT_MDL_NSGN   0
#define FIX_QD_ANGVEL_OUT_MDL_NSPHIN 0
#define FIX_QD_ANGVEL_OUT_MDL_NSPHI0 0
#define FIX_QD_ANGVEL_OUT_MDL_NSBXN  0
#define FIX_QD_ANGVEL_OUT_MDL_NS     0
#define FIX_QD_ANGVEL_OUT_MDL_NS0    0
#define FIX_QD_ANGVEL_OUT_MDL_NSN    0
#define FIX_QD_ANGVEL_OUT_MDL_NG     0
#define FIX_QD_ANGVEL_OUT_MDL_NBXN   3
#define FIX_QD_ANGVEL_OUT_MDL_NGN    0
#define FIX_QD_ANGVEL_OUT_MDL_NY0    14
#define FIX_QD_ANGVEL_OUT_MDL_NY     14
#define FIX_QD_ANGVEL_OUT_MDL_NYN    10
#define FIX_QD_ANGVEL_OUT_MDL_N      20
#define FIX_QD_ANGVEL_OUT_MDL_NH     0
#define FIX_QD_ANGVEL_OUT_MDL_NHN    0
#define FIX_QD_ANGVEL_OUT_MDL_NH0    0
#define FIX_QD_ANGVEL_OUT_MDL_NPHI0  0
#define FIX_QD_ANGVEL_OUT_MDL_NPHI   0
#define FIX_QD_ANGVEL_OUT_MDL_NPHIN  0
#define FIX_QD_ANGVEL_OUT_MDL_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct fix_qd_angvel_out_mdl_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_e_fun;
    external_function_external_param_casadi cost_y_e_fun_jac_ut_xt;


    // constraints







} fix_qd_angvel_out_mdl_solver_capsule;

ACADOS_SYMBOL_EXPORT fix_qd_angvel_out_mdl_solver_capsule * fix_qd_angvel_out_mdl_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_free_capsule(fix_qd_angvel_out_mdl_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_create(fix_qd_angvel_out_mdl_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_reset(fix_qd_angvel_out_mdl_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of fix_qd_angvel_out_mdl_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_create_with_discretization(fix_qd_angvel_out_mdl_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_update_time_steps(fix_qd_angvel_out_mdl_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_update_qp_solver_cond_N(fix_qd_angvel_out_mdl_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_update_params(fix_qd_angvel_out_mdl_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_update_params_sparse(fix_qd_angvel_out_mdl_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_set_p_global_and_precompute_dependencies(fix_qd_angvel_out_mdl_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_solve(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_setup_qp_matrices_and_factorize(fix_qd_angvel_out_mdl_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_free(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void fix_qd_angvel_out_mdl_acados_print_stats(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int fix_qd_angvel_out_mdl_acados_custom_update(fix_qd_angvel_out_mdl_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *fix_qd_angvel_out_mdl_acados_get_nlp_in(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *fix_qd_angvel_out_mdl_acados_get_nlp_out(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *fix_qd_angvel_out_mdl_acados_get_sens_out(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *fix_qd_angvel_out_mdl_acados_get_nlp_solver(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *fix_qd_angvel_out_mdl_acados_get_nlp_config(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *fix_qd_angvel_out_mdl_acados_get_nlp_opts(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *fix_qd_angvel_out_mdl_acados_get_nlp_dims(fix_qd_angvel_out_mdl_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *fix_qd_angvel_out_mdl_acados_get_nlp_plan(fix_qd_angvel_out_mdl_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_fix_qd_angvel_out_mdl_H_
