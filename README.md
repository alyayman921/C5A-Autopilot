# Flight Simulator Software, Developed in C++
a a continuation to the AER4420 Course in my Final year of aerospace engineering atCcairo University>
during the course i was tasked to create an autopilot system for the Lockheed Martin's C5A Aircraft using Matlab/Simulink, this is C++ implementation for the same linearized aircraft dynamics around a certain flight condition with the same controllers.
[Linkedin 1/3](https://www.linkedin.com/posts/3lymn_autopilot-13-model-based-development-of-ugcPost-7483422524295442432-yo6z/?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD8q71sBYWkieo0AnWkqPzl4N3JYOQFtBKU) [Linkedin 2/3](https://www.linkedin.com/posts/3lymn_autopilot-23-%F0%9D%9F%AD%F0%9D%9F%AE%F0%9D%98%85-%F0%9D%97%A7%F0%9D%97%B5%F0%9D%97%B2-%F0%9D%97%A3%F0%9D%97%B2%F0%9D%97%BF%F0%9D%97%B3%F0%9D%97%BC%F0%9D%97%BF%F0%9D%97%BA%F0%9D%97%AE%F0%9D%97%BB%F0%9D%97%B0%F0%9D%97%B2-ugcPost-7484514915890663424-gycB/?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD8q71sBYWkieo0AnWkqPzl4N3JYOQFtBKU)
[Linkedin 3/3](https://www.linkedin.com/posts/3lymn_autopilot-33-sooo-this-is-the-final-share-7488380808546938880-Xzcs/?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD8q71sBYWkieo0AnWkqPzl4N3JYOQFtBKU)


![Release 1.41](https://github.com/alyayman921/Docs-Flight-Sim/blob/22f7b473e203b28654dda597e06792ed1c77be67/V1.41/Release1.41.gif)


# Features
- Rigid Body Dynamics simulator with RK4 solver
- Easy Controller Implementation with in the house Transfer Function library
- Linear Simulator Implementation
- Mode for STM32 board as Controller logic
- Mode STM32 Standalone Linear Simulator
- For Any Aircraft, provided you add your dynamics and controllers

---

<p align="center">
  <img src="https://github.com/alyayman921/Docs-Flight-Sim/blob/22f7b473e203b28654dda597e06792ed1c77be67/V2/gfx.gif" alt="Speed Demo V2" />
</p>

---

## Build From Source
*Dependencies:*
<!-- - [XLSX I/O C library](https://github.com/brechtsanders/xlsxio). -->
<!-- - C++ Eigen, most likely preinstalled in your distro, can be downloaded and put in the project directory, but you'll have to do some manual renaming in the header files. -->
<!-- - [Libserial](https://github.com/crayzeewulf/libserial), Self Explanatory, not available for windows. -->
No Dependencies!, Unless you want Serial communication or you want to use your own aircraft.
## **Windows Guide**
Serial Library is not for windows
[Download Latest Release](https://github.com/alyayman921/Cpp-Flight-Simulator/releases

it is critical to run this from a terminal in windows, do this in the terminal
```powershell
.\FlightSimulator.exe
```


## **Linux Guide:**
[Download Latest Release](https://github.com/alyayman921/Cpp-Flight-Simulator/releases

**The release binaries are compiled with gcc-9 to ensure compatibility, follow this guide for best performance.**

```bash
mkdir /tmp/FlightSim
cd /tmp/FlightSim
git clone --depth 1 https://github.com/alyayman921/Cpp-Flight-Simulator.git
cd Cpp*
mkdir -p Release/Release_Linux_x64
make clean
make release
make install
echo "Now Run it with 'FlightSimulator --help' to Get started, if you don't see it, add ~/.local/bin to path"
```

---

## About This Simulator
> [!WARNING]
> this model does not approximate the continous time system like matlab would, notice the dynamics change with different time steps.

#### **Small 1ms Timesteps C++ vs Matlab**

![Cpp vs Matlab at 0.001](Documentation/V0.1/CppVsMatlab150@ts0.001.png)

---

#### **Bigger  10ms Timesteps C++ vs Matlab**

![Cpp vs Matlab at 0.001](Documentation/V0.1/CppVsMatlab150@ts0.01.png)

## Controller Implementation Time Scheme
- This is a pseudo-Continuous Time project, using same controllers from the S domain like this:
if a state is to be derived (multiplied by s), for the first time step we set the state_derivative = sum of non derivated states, next time step you get the state by doing state += state_derivative * dt, look at the [tf.hpp](tf.hpp) file for more info.

## TODO
- [x] Make a Generalized Transfer Function class.
- [x] Deploy controllers to an stm32 based embedded board.
- [x] Remove Libeigen and xlsxio
- [ ] Attach a rendering GUI for the Simulator.
- [ ] Publish to AUR

> [!NOTE]
> ## Acknowledgement
> Thanks To A.Kotb For testing on Ubuntu 22.04


## AI DISCLAIMER
> [!IMPORTANT]
> Ai was used in this project to do the logging and the handling of files, it was also used in developing the rk4 solver and linking it to the Rigid Body Dynamics equations.
> this is a learning project, i'm trying to enhance my C++ skills so there's no point of prompting the ai to do it for me.
> ai was used too in the STM32 part of the project, to help with the uncharted Drivers territory and do the donkey work that was done before, for example after setting up the communication bridge via USB CDC, ai converted the floats to certain amount of characters to send from the c++ code to stm, you can probably tell whose idea this was, it also implemented the same controllers with the euler method, since it choked on it's own saliva while trying to use the digital controllers.
