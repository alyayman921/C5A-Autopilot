#include "RBDEqns.hpp"

  RBDSolve::RBDSolve(aircraft_data &ac, Eigen::Matrix<double,4,1>* Controls,flight_path *str_h ,Eigen::Matrix<double,9,1>* states){
          this->ac = ac; this->g = ac.g; this->m = ac.m; this->I = ac.Inertia;this->z0=ac.z0;
          this->F_g0 = ac.mg0; this->v = ac.V0;
          this->omega = ac.omega0; this->euler = ac.euler0;
          this->SD = ac.SD; this->CD = ac.CD;
          this->w_dot_state = 0;
          this->Controls = Controls;
          this->states=states;
          this->str_h=str_h;
          F_aero.setZero(); F_grav.setZero();
          F_b.setZero(); M_total.setZero();
          str_h->h=-ac.z0;
  }
  void RBDSolve::eulerToRotationMatrix(){
      double phi = euler(0); double c_phi = std::cos(phi); double s_phi = std::sin(phi);
      double theta = euler(1); double c_theta = std::cos(theta); double s_theta = std::sin(theta);
      double psi = euler(2); double c_psi = std::cos(psi); double s_psi = std::sin(psi);

      R << c_psi * c_theta,
            c_psi * s_theta * s_phi - s_psi * c_phi,
            c_psi * s_theta * c_phi + s_psi * s_phi,

            s_psi * c_theta,
            s_psi * s_theta * s_phi + c_psi * c_phi,
            s_psi * s_theta * c_phi - c_psi * s_phi,

          -s_theta,
            c_theta * s_phi,
            c_theta * c_phi;
  }

  Eigen::Matrix<double,9,1> RBDSolve::RBDEquations(Eigen::Matrix<double,9,1> y){
      v = y.segment<3>(0);
      omega = y.segment<3>(3);
      euler = y.segment<3>(6);

      // calculate forces and moments
      delta_v = v- ac.V0;
      delta_omega = omega-ac.omega0;

      // Build delta_states
      delta_y(0) = delta_v(0);
      delta_y(1) = delta_v(1);
      delta_y(2) = delta_v(2);
      delta_y(3) = delta_omega(0);
      delta_y(4) = delta_omega(1);
      delta_y(5) = delta_omega(2);
      delta_y(6) = w_dot_state;

      // Calculate Aerodynamic Accelerations
      Aerodynamic_accel = SD * delta_y + CD * (*Controls);

      // Compute aerodynamic forces
      for (i = 0; i < 3; i++){
          delta_F[i] = Aerodynamic_accel[i] * m;
          F_aero[i] = delta_F[i];  // Store aerodynamic forces
      }
      // Compute moments
      for (i = 3; i < 6; i++){
          delta_M[i-3] = Aerodynamic_accel[i] * I.coeff(i-3, i-3);
      }
      M_total = delta_M;

      // Compute gravitational forces
      F_grav(0) = -m * g * sin(euler[1]) - F_g0[0];
      F_grav(1) =  m * g * cos(euler[1]) * sin(euler[0]) - F_g0[1];
      F_grav(2) =  m * g * cos(euler[1]) * cos(euler[0]) - F_g0[2];

      // Total body forces
      F_b = delta_F + F_grav;

      // linear newton
      v_dot = F_b / m - omega.cross(v); // Matlab 6DOF Abb

      // Angular Newton
      delta_omega_dot = delta_M - omega.cross(I * omega);
      delta_omega_dot = I.ldlt().solve(delta_omega_dot);

      // Euler Kinematics
      cos_theta = std::cos(euler[1]);
      if (std::abs(cos_theta) < eps) cos_theta = eps;

      J << 1, sin(euler[0])*tan(euler[1]), cos(euler[0])*tan(euler[1]),
            0, cos(euler[0]),               -sin(euler[0]),
            0, sin(euler[0])/cos_theta,      cos(euler[0])/cos_theta;
      euler_dot = J * omega;

      // Build y_dot (9x1)
      y_dot(0) = v_dot(0);
      y_dot(1) = v_dot(1);
      y_dot(2) = v_dot(2);
      y_dot(3) = delta_omega_dot(0);
      y_dot(4) = delta_omega_dot(1);
      y_dot(5) = delta_omega_dot(2);
      y_dot(6) = euler_dot(0);
      y_dot(7) = euler_dot(1);
      y_dot(8) = euler_dot(2);


      return y_dot;
  }

  void RBDSolve::rk4Solver(){
      double t = step * dt;
      double t_half = t + dt/2.0f;
      double t_full = t + dt;
      y = *states;
      Eigen::Matrix<double,9,1> k1, k2, k3, k4;

      k1 = RBDSolve::RBDEquations(y);
      k2 = RBDSolve::RBDEquations(y + 0.5f * dt * k1);
      k3 = RBDSolve::RBDEquations(y + 0.5f * dt * k2);
      k4 = RBDSolve::RBDEquations(y + dt * k3);
      y = y + dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f; // New State Vector
      // h_calc(y);
      str_h->alpha=std::atan2(y(2),y(0));
      str_h->v_tot=std::hypot(y(0),y(2));
      str_h->beta=std::atan2(y(1),str_h->v_tot);
      str_h->gamma=y(7)-str_h->alpha;
      str_h->delta_h_dot=str_h->v_tot*std::sin(str_h->gamma);
      str_h->h+=dt*str_h->delta_h_dot;
      w_dot_state = v_dot(2);
      *states=y;
  }
