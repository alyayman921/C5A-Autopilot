#include "RBDEqns.hpp"

static void cross3(struct Matrix* out, struct Matrix* a, struct Matrix* b) {
    out->data[0][0] = a->data[1][0] * b->data[2][0] - a->data[2][0] * b->data[1][0];
    out->data[1][0] = a->data[2][0] * b->data[0][0] - a->data[0][0] * b->data[2][0];
    out->data[2][0] = a->data[0][0] * b->data[1][0] - a->data[1][0] * b->data[0][0];
}

/* Solves A * x = b for a 3x3 matrix A (Gaussian elimination, partial pivot). */
static void solve3(struct Matrix* x, struct Matrix* A, struct Matrix* b) {
    float a[3][3], bb[3];
    for (int r = 0; r < 3; r++) {
        bb[r] = b->data[r][0];
        for (int c = 0; c < 3; c++) a[r][c] = A->data[r][c];
    }
    for (int r = 0; r < 3; r++) {
        int p = r;
        for (int k = r + 1; k < 3; k++)
            if (std::fabs(a[k][r]) > std::fabs(a[p][r])) p = k;
        if (p != r) {
            for (int c = 0; c < 3; c++) { float t = a[r][c]; a[r][c] = a[p][c]; a[p][c] = t; }
            float t = bb[r]; bb[r] = bb[p]; bb[p] = t;
        }
        for (int k = r + 1; k < 3; k++) {
            float f = a[k][r] / a[r][r];
            for (int c = r; c < 3; c++) a[k][c] -= f * a[r][c];
            bb[k] -= f * bb[r];
        }
    }
    float X[3];
    for (int r = 2; r >= 0; r--) {
        float s = bb[r];
        for (int c = r + 1; c < 3; c++) s -= a[r][c] * X[c];
        X[r] = s / a[r][r];
    }
    for (int r = 0; r < 3; r++) x->data[r][0] = X[r];
}

RBDSolve::RBDSolve(aircraft_data &ac, struct Matrix* Controls, flight_path *str_h ,struct Matrix* states){
        this->ac = &ac;
        this->m = ac.m; this->g = ac.g; this->z0 = ac.z0;
        this->v = vector(3);       mat_copy(&this->v, &ac.V0);
        this->omega = vector(3);   mat_copy(&this->omega, &ac.omega0);
        this->euler = vector(3);   mat_copy(&this->euler, &ac.euler0);
        this->I = matrix(3,3);       mat_copy(&this->I, &ac.Inertia);
        this->F_g0 = vector(3);    mat_copy(&this->F_g0, &ac.mg0);
        this->SD = matrix(6,7);      mat_copy(&this->SD, &ac.SD);
        this->CD = matrix(6,4);      mat_copy(&this->CD, &ac.CD);
        this->w_dot_state = 0;
        this->Controls = Controls;
        this->states = states;
        this->str_h = str_h;
        this->v_dot = vector(3);
        this->delta_omega_dot = vector(3);
        this->euler_dot = vector(3);
        this->J = matrix(3,3);
        this->Aerodynamic_accel = vector(6);
        this->delta_v = vector(3);
        this->delta_omega = vector(3);
        this->delta_F = vector(3);
        this->delta_M = vector(3);
        this->y = vector(9);
        this->y_dot = vector(9);
        this->delta_y = vector(7);
        this->F_aero = vector(3);
        this->F_grav = vector(3);
        this->F_b = vector(3);
        this->M_total = vector(3);
        mat_set_zero(&this->F_aero); mat_set_zero(&this->F_grav);
        mat_set_zero(&this->F_b);    mat_set_zero(&this->M_total);
        str_h->h = -ac.z0;
}

RBDSolve::~RBDSolve(){
    mat_free_memory(&v); mat_free_memory(&v_dot); mat_free_memory(&omega);
    mat_free_memory(&delta_omega_dot); mat_free_memory(&euler); mat_free_memory(&euler_dot);
    mat_free_memory(&J); mat_free_memory(&I); mat_free_memory(&SD); mat_free_memory(&CD);
    mat_free_memory(&Aerodynamic_accel); mat_free_memory(&delta_v); mat_free_memory(&delta_omega);
    mat_free_memory(&delta_F); mat_free_memory(&delta_M); mat_free_memory(&y); mat_free_memory(&y_dot);
    mat_free_memory(&delta_y); mat_free_memory(&F_aero); mat_free_memory(&F_grav);
    mat_free_memory(&F_b); mat_free_memory(&F_g0); mat_free_memory(&M_total);
}

void RBDSolve::RBDEquations(struct Matrix y, struct Matrix* out){
    for (i = 0; i < 3; i++){
        v.data[i][0] = y.data[i][0];
        omega.data[i][0] = y.data[i+3][0];
        euler.data[i][0] = y.data[i+6][0];
    }

    // calculate forces and moments
    mat_sub(&delta_v, &v, &ac->V0);
    mat_sub(&delta_omega, &omega, &ac->omega0);

    // Build delta_states
    delta_y.data[0][0] = delta_v.data[0][0];
    delta_y.data[1][0] = delta_v.data[1][0];
    delta_y.data[2][0] = delta_v.data[2][0];
    delta_y.data[3][0] = delta_omega.data[0][0];
    delta_y.data[4][0] = delta_omega.data[1][0];
    delta_y.data[5][0] = delta_omega.data[2][0];
    delta_y.data[6][0] = w_dot_state;

    // Calculate Aerodynamic Accelerations
    struct Matrix tmp1 = {0};
    mat_mult(SD, delta_y, &tmp1);
    struct Matrix tmp2 = {0};
    mat_mult(CD, *Controls, &tmp2);
    mat_add(&Aerodynamic_accel, &tmp1, &tmp2);
    mat_free_memory(&tmp1); mat_free_memory(&tmp2);

    // Compute aerodynamic forces
    for (i = 0; i < 3; i++){
        delta_F.data[i][0] = Aerodynamic_accel.data[i][0] * m;
        F_aero.data[i][0] = delta_F.data[i][0];  // Store aerodynamic forces
    }
    // Compute moments
    for (i = 3; i < 6; i++){
        delta_M.data[i-3][0] = Aerodynamic_accel.data[i][0] * I.data[i-3][i-3];
    }
    mat_copy(&M_total, &delta_M);

    // Compute gravitational forces
    F_grav.data[0][0] = -m * g * std::sin(euler.data[1][0]) - F_g0.data[0][0];
    F_grav.data[1][0] =  m * g * std::cos(euler.data[1][0]) * std::sin(euler.data[0][0]) - F_g0.data[1][0];
    F_grav.data[2][0] =  m * g * std::cos(euler.data[1][0]) * std::cos(euler.data[0][0]) - F_g0.data[2][0];

    // Total body forces
    mat_add(&F_b, &delta_F, &F_grav);

    // linear newton
    struct Matrix Fb_scaled = vector(3);
    mat_scalar_mul(&Fb_scaled, &F_b, 1.0f / (float)m);
    struct Matrix cross1 = vector(3);
    cross3(&cross1, &omega, &v);
    mat_sub(&v_dot, &Fb_scaled, &cross1);
    mat_free_memory(&Fb_scaled); mat_free_memory(&cross1);

    // Angular Newton
    struct Matrix Iomega = {0};
    mat_mult(I, omega, &Iomega);
    struct Matrix cross2 = vector(3);
    cross3(&cross2, &omega, &Iomega);
    struct Matrix diff = vector(3);
    mat_sub(&diff, &delta_M, &cross2);
    solve3(&delta_omega_dot, &I, &diff);
    mat_free_memory(&Iomega); mat_free_memory(&cross2); mat_free_memory(&diff);

    // Euler Kinematics
    cos_theta = std::cos(euler.data[1][0]);
    if (std::fabs(cos_theta) < eps) cos_theta = eps;

    J.data[0][0] = 1; J.data[0][1] = std::sin(euler.data[0][0]) * std::tan(euler.data[1][0]); J.data[0][2] = std::cos(euler.data[0][0]) * std::tan(euler.data[1][0]);
    J.data[1][0] = 0; J.data[1][1] = std::cos(euler.data[0][0]);                                 J.data[1][2] = -std::sin(euler.data[0][0]);
    J.data[2][0] = 0; J.data[2][1] = std::sin(euler.data[0][0]) / cos_theta;                       J.data[2][2] = std::cos(euler.data[0][0]) / cos_theta;
    mat_mult(J, omega, &euler_dot);

    // Build y_dot (9x1)
    for (i = 0; i < 3; i++) y_dot.data[i][0] = v_dot.data[i][0];
    for (i = 0; i < 3; i++) y_dot.data[i+3][0] = delta_omega_dot.data[i][0];
    for (i = 0; i < 3; i++) y_dot.data[i+6][0] = euler_dot.data[i][0];

    mat_copy(out, &y_dot);
}

void RBDSolve::rk4Solver(){
    mat_copy(&y, states);
    struct Matrix k1 = vector(9); RBDEquations(y, &k1);

    struct Matrix tmp = vector(9);
    mat_scalar_mul(&tmp, &k1, 0.5f * (float)dt);
    struct Matrix y2 = vector(9); mat_add(&y2, &y, &tmp);
    struct Matrix k2 = vector(9); RBDEquations(y2, &k2);

    mat_scalar_mul(&tmp, &k2, 0.5f * (float)dt);
    struct Matrix y3 = vector(9); mat_add(&y3, &y, &tmp);
    struct Matrix k3 = vector(9); RBDEquations(y3, &k3);

    mat_scalar_mul(&tmp, &k3, (float)dt);
    struct Matrix y4 = vector(9); mat_add(&y4, &y, &tmp);
    struct Matrix k4 = vector(9); RBDEquations(y4, &k4);

    struct Matrix sum = vector(9);
    mat_scalar_mul(&tmp, &k2, 2.0f);
    mat_add(&sum, &k1, &tmp);
    mat_scalar_mul(&tmp, &k3, 2.0f);
    mat_add(&sum, &sum, &tmp);
    mat_add(&sum, &sum, &k4);
    mat_scalar_mul(&sum, &sum, (float)dt / 6.0f);
    mat_add(&y, &y, &sum);

    mat_free_memory(&k1); mat_free_memory(&k2); mat_free_memory(&k3); mat_free_memory(&k4);
    mat_free_memory(&tmp); mat_free_memory(&y2); mat_free_memory(&y3); mat_free_memory(&y4); mat_free_memory(&sum);

    str_h->alpha = std::atan2(y.data[2][0], y.data[0][0]);
    str_h->v_tot = std::hypot(y.data[0][0], y.data[2][0]);
    str_h->beta = std::atan2(y.data[1][0], str_h->v_tot);
    str_h->gamma = y.data[7][0] - str_h->alpha;
    str_h->delta_h_dot = str_h->v_tot * std::sin(str_h->gamma);
    str_h->h += dt * str_h->delta_h_dot;
    w_dot_state = v_dot.data[2][0];
    mat_copy(states, &y);
}
