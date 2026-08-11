#include "flightsim.hpp"

int main(int argc, char* argv[]) {
/*------------------------- Import Files ----------------------------------------*/
    readxlsx(filename);
    aircraft_data c5a = readAircraft();

    if (!readControlsFromFile("meta/controls.txt", Controls, dt, tfinal)) {
        std::cerr << "Failed to read meta/controls.txt, Using defaults..." << std::endl;
        Controls << 0.0, 0.0, 1000.0, 0.05;
        dt = 0.01;
        tfinal = 100.0;
    }
    if (!Autopiloted){std::cout << "Controls loaded: " << Controls.transpose() << std::endl;}
    std::cout << "Timestep: " << dt << " s" << std::endl;
    std::cout << "Final time: " << tfinal << " s" << std::endl;
    std::cout << "Number of steps: " << (int)(tfinal / dt) << std::endl;

/*------------------------- Handle Inputs----------------------------------------*/
    if (argc >= 2){
        std::string arg = argv[1];
        if (arg == "--help") {
            std::cout<< "\nFlight Simulator for Lockheed Martin's C5A Aircraft\n";
            std::cout<< "";
            std::cout<< "\n";
            std::cout<< "Arguments    Usage\n";
            std::cout<< "---------    ---------------------------------------------------------\n";
            std::cout<< "--help       print this message\n";
            std::cout<< "--lin        switch to linear statespace simulator\n";
            std::cout<< "--loop       prevent the program from exiting after solving\n";
            std::cout<< "--manual     read the control commands from the textfile controls.txt\n";
            std::cout<< "--pitch      overrides altitude loop straight to pitch control\n";
            std::cout<< "--roll       overrides heading loop straight to roll control\n\n";
            std::cout<< "--------------------- STM32F103C8T + Linux Only ----------------------\n";
            std::cout<< "--ext        use external Micorcontroller as aircraft controller\n";
            std::cout<< "--onboard    solve the linear sim on the STM\n";
            return 0;
        }
        for(int i=0;i<argc;i++){
            arg=argv[i];
            if (arg == "test"){
              mode='s'; // skip
              commands.set_alt=1000-c5a.z0;
              commands.set_heading=30*deg2rad;
              commands.set_vel=80;commands.set_vel += c5a.V0(0);
              std::cout<<"alt set: "<<commands.set_alt<<'\n';
              std::cout<<"heading set: "<<commands.set_heading*rad2deg<<'\n';
              std::cout<<"vel set: "<<commands.set_vel<<'\n';
              break;
            }
            if (arg == "--loop") {
                std::cout<<"Prevent from Exit : True\n";
                loop = true;
            }
            if (arg == "--manual") {
                std::cout<<"Applying Control actions directly from text file\n";
                Autopiloted = false;
            }
            if (arg == "--pitch") {
                std::cout<<"Skipping altitude loop\n";
                commands.alt_override = true;
            }
            if (arg == "--roll") {
                std::cout<<"Skipping heading loop\n";
                commands.head_override = true;
            }
            if (arg == "--ext") {
                std::cout<<"External Controller\n";
                commands.ext_controller = true;
            }
            if (arg == "--lin") {
                std::cout<<"Linear Simulator\n";
                commands.linear= true;
            }
            if (arg == "--onboard") {
                std::cout<<"External Linear Simulator\n";
                commands.onboard= true;
                commands.ext_controller = true;
            }
        }
    }
    if(mode!='s'){
        if(commands.alt_override){
            std::cout<<"Pitch Autopilot change (Degrees): ";
            std::cin >> commands.set_pitch; commands.set_pitch=commands.set_pitch*deg2rad;
        }else{
            std::cout<<"Altitude Change (ft): ";
            std::cin >> commands.set_alt ;commands.set_alt -= c5a.z0;
        }
        if(commands.head_override){
            std::cout<<"Roll Autopilot change (Degrees): ";
            std::cin >> commands.set_roll; commands.set_roll=commands.set_roll*deg2rad;
        }else{
            std::cout<<"Heading Change (Degrees): ";
            std::cin >> commands.set_heading; commands.set_heading=commands.set_heading*deg2rad;
        }
      std::cout<<"Velocity Autopilot change (ft/s): ";
      std::cin >> commands.set_vel ;commands.set_vel += c5a.V0(0);
    }


/*------------------------- Problem Initialization ----------------------------------------*/
    // Initial state vector: [uvw, pqr, euler(3)]
    Eigen::Matrix<double, 9, 1> initial_state;
    initial_state << c5a.V0(0), c5a.V0(1), c5a.V0(2),
                      c5a.omega0(0), c5a.omega0(1), c5a.omega0(2),
                      c5a.euler0(0), c5a.euler0(1), c5a.euler0(2);
    int N_steps = (int)(tfinal / dt);
    Eigen::Matrix<double,9,1> results;
    results=initial_state;
    // Initialize the controller
    controller c(&Controls,&results,&c5a,&str_h,&commands);
    auto prev =std::chrono::steady_clock::now();

#ifdef USE_SERIAL
    if (commands.onboard){
        c.onboard_sim();
    } else
#endif


/*------------------------- Solving Linear ----------------------------------------*/
  std::cout<<"<Simulation Progress> \n";

  if (commands.linear){
    fullLinear lin(&Controls,&c5a, &str_h,&commands,&results);
    for(step;step<N_steps;step++){
      lin.solve();
      c.updateControllers();
      if(step%10==0){
          progress_percent=1+((float)step/(float)N_steps)*20;
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

/*------------------------- Solving NONLinear ----------------------------------------*/
  }else if(!commands.onboard){
    RBDSolve RBD(c5a, &Controls,&str_h, &results);
    for(step;step<N_steps;step++){
      RBD.rk4Solver();
      c.updateControllers();
    if(step%10==0){
        progress_percent=1+((float)step/(float)N_steps)*20;
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
  }


/*-------------------------Results ----------------------------------------*/
    std::cout << "\n=== Final State (t=" << tfinal << "s) ===" << std::endl;
    std::cout << "Velocity ft/s (v_x, v_y, v_z): "
              << results(0) << ", " << results(1) << ", " << results(2) << std::endl;

    std::cout << "Euler angles in Degrees (phi, theta, psi): "
              << (float)results(6)*rad2deg << ", " << (float)results(7)*rad2deg << ", " << (float)results(8)*rad2deg << std::endl;

    std::cout << "Final Altitude: "<<str_h.h<<"\n";


#ifdef USE_SERIAL
    if (commands.ext_controller&& !commands.onboard){
        std::cout << "Missed serial ticks: " << c.missed_ticks << "/" << (int)(tfinal / dt) << "\n";
    }
#endif
    auto now =std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed_seconds{now-prev};
    float elapsed= (float) elapsed_seconds.count();
    float RTF = (double)tfinal/elapsed_seconds.count();
    std::cout<<"Simulation Finished in "<<elapsed<<" With a RTF " <<RTF<<std::endl;

    if (loop){
        std::cout<<"Press Ctrl+C to Exit\n";
        std::cin >> dt;
    }
    return 0;
}
