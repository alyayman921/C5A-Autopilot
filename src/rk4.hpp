#pragma once

#include <cmath>
#include <cstdlib>
#include "RBDEqns.hpp"
#include "logger.hpp"
#include "controller.hpp"
class rk4{
    private:
        int i;
        double dt;
        double tfinal;
        int N_steps;
        int *step;
        double *time_v;
        // Loggers
        DataLogger stateLogger;
        DataLogger forceLogger;
        DataLogger accelLogger;
        DataLogger momentLogger;
        DataLogger controlLogger;
        bool loggersInitialized,logging;

    public:
        Eigen::Matrix<double,9,1> *state_history;
        rk4(double dt, double tfinal,int *step,bool logging) {
            this->dt = dt;
            this->tfinal = tfinal;
            this->N_steps = (int)(tfinal / dt);
            this->time_v = (double*)malloc(N_steps * sizeof(double));
            this->state_history = (Eigen::Matrix<double,9,1>*)malloc((N_steps+1) * sizeof(Eigen::Matrix<double,9,1>));
            this->loggersInitialized = false;
            this->step=step;
            this->logging=logging;
            for (int i = 0; i < N_steps; i++) {
                time_v[i] = i * dt;
            }
        }
        ~rk4() {
            // Loggers will be closed automatically by their destructors
        }
        void initializeLoggers() {
            if (loggersInitialized) return;
            // Initialize state logger
            stateLogger.init("states");
            std::vector<std::string> stateHeaders = {
                "time", "vx", "vy", "vz", "p", "q", "r", "phi", "theta", "psi"
            };
            stateLogger.writeHeader(stateHeaders);
            // Initialize force logger
            forceLogger.init("forces");
            std::vector<std::string> forceHeaders = {
                "time",
                "F_aero_x", "F_aero_y", "F_aero_z",
                "F_grav_x", "F_grav_y", "F_grav_z",
                "F_total_x", "F_total_y", "F_total_z"
            };
            forceLogger.writeHeader(forceHeaders);
            // Initialize aerodynamic acceleration logger
            accelLogger.init("aerodynamic_accel");
            std::vector<std::string> accelHeaders = {
                "time", "ax", "ay", "az", "alpha", "beta", "gamma"
            };
            accelLogger.writeHeader(accelHeaders);
             momentLogger.init("moments");
            std::vector<std::string> momentHeaders = {
                "time", "Mx", "My", "Mz"
            };
            momentLogger.writeHeader(momentHeaders);
            controlLogger.init("controls");
            std::vector<std::string> controlHeaders = {
                "time", "da", "de", "dth", "dr"
            };
            controlLogger.writeHeader(controlHeaders);
            loggersInitialized = true;
            //std::cout << "All loggers initialized successfully!" << std::endl;
        }
        void resultsPointer(controller &con_obj){ // should be called reference but idc
            // send the adress of results to controller
            con_obj.rk4_pointers(state_history);
        }

        Eigen::Matrix<double,9,1>* rk4_solver(RBDsolve &RBDobj, controller &con_obj,flight_path &str_h, Eigen::Matrix<double,9,1> states0){
            Eigen::Matrix<double,9,1> y = states0;
            Eigen::Matrix<double,9,1> k1, k2, k3, k4;

            // Initialize loggers
            if (logging){
                initializeLoggers();
                // Log initial state (t=0)
                state_history[0] = states0;
                RBDobj.Equations(y, 0.0);

                stateLogger.logStates(0.0, y);
                forceLogger.logForces(0.0, RBDobj.getAeroForces(), RBDobj.getGravForces(), RBDobj.getTotalForces());
                accelLogger.logWithTime(0.0, RBDobj.getAerodynamicAccel());
                momentLogger.logMoments(0.0, RBDobj.getTotalMoments());
            }else{
                state_history[0] = states0;
                RBDobj.Equations(y, 0.0);

            }

            std::cout<<"<Simulation Progress> \n";
            int progress_percent=0;
            for (*step = 0; *step < N_steps; (*step)++) {

                double t = *step * dt;
                double t_half = t + dt/2.0f;
                double t_full = t + dt;


                // k1 = f(t, y)
                k1 = RBDobj.Equations(y, t);

                // k2 = f(t + dt/2, y + dt*k1/2)
                k2 = RBDobj.Equations(y + 0.5f * dt * k1, t_half);

                // k3 = f(t + dt/2, y + dt*k2/2)
                k3 = RBDobj.Equations(y + 0.5f * dt * k2, t_half);

                // k4 = f(t + dt, y + dt*k3)
                k4 = RBDobj.Equations(y + dt * k3, t_full);

                // y_{n+1} = y_n + dt/6*(k1 + 2k2 + 2k3 + k4)
                y = y + dt * (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;

                // Update w_dot_state using v_dot(2)
                RBDobj.Equations(y, t + dt);
                RBDobj.updatewdot(RBDobj.getVDotZ());
                RBDobj.h_calculation(y,dt,str_h);
                //std::cout<<"current H"<<str_h.h<<" "<< "Current h_dot" <<str_h.delta_h_dot<<std::endl;

                // Store state
                state_history[*step + 1] = y;
                // update the controllers after solving
                con_obj.pitch_controller();
                con_obj.velocity_controller();
                con_obj.altitude_controller();
                con_obj.roll_controller();
                con_obj.yaw_controller();
                con_obj.send_states();
                // Log data at this timestep
                double current_time = (*step + 1) * dt;
                if (logging){

                stateLogger.logStates(current_time, y);
                forceLogger.logForces(current_time,
                                     RBDobj.getAeroForces(),
                                     RBDobj.getGravForces(),
                                     RBDobj.getTotalForces());
                accelLogger.logWithTime(current_time, RBDobj.getAerodynamicAccel());
                momentLogger.logMoments(current_time, RBDobj.getTotalMoments());
                controlLogger.logWithTime(current_time, *con_obj.get_controls());
                }
                // Simulation Progress animation
                if(*step%10==0){
                    progress_percent=1+((float)*step/(float)N_steps)*20;
                    std::cout<<"[";
                    int j,k;
                    for (j=0;j<progress_percent;j++){
                        std::cout<<"|";
                    }
                    for (k=20-j;k>0;k--){
                        std::cout<<" ";
                    }
                        std::cout<<"]\r";
                    }
            }

            return state_history;
        }

        void free_results() {
            free(time_v);
            free(state_history);
            // Loggers will close automatically in their destructor
            // But you can explicitly close them if needed
            if (logging){
                stateLogger.close();
                forceLogger.close();
                accelLogger.close();
                momentLogger.close();
                controlLogger.close();
            }
        }
};
