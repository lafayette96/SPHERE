# SPHERE Spherical Robot Project

This is a repository that stores source code for spherical robot "SPHERE".  
The robot was developed and built by the students of AGH University of Science and Technology.   
Kraków, 2018/2019. 

![SPHERE Spherical Robot](/Pictures/sphere.png "SPHERE")

## How to begin:

- To use DualShock 4 controller:
  - Open DS4 Windows and enable laptop's bluetooth 
  - Switch on the DS4 controller
  - Connect the controller to the app
  - Switch on the robot
  - Execute `SPHERE_DS4_Control.py`
&nbsp;    

- To use PC GUI application: 
  - Switch on the robot
  - Execute `SPHERE_PC_Control.py`
&nbsp;  

## How to control the robot with PC app:

![PC Control app](/Pictures/GUI3.PNG "PC Control app")

* `Throttle` section allows the user to set the desired value of the robot’s velocity.  
* `Steering` section allows the user to lean the pendulum and turn the robot sideways.  
* `Controllers` section lets the user choose what controller should be used.  
* `STOP` button resets throttle and steering values to 0. It is a safety button.    
&nbsp;    
  
## How to control the robot with DS4:

![DualShock 4 controls](/Pictures/DS4_resized.PNG "DualShock 4")
  
* Analog joystick `1` allows to set a pendulum inclination and turn the robot.  
* L2 button marked as `2` controls the robot’s backward speed.  
* R2 button marked as `3` controls robot’s forward speed.  
* Buttons marked as `4` allow the user to change controller type. Controller selection:   
&nbsp;&nbsp;&nbsp;&nbsp; `X` - no controller  
&nbsp;&nbsp;&nbsp;&nbsp; `square` - PID controller   
&nbsp;&nbsp;&nbsp;&nbsp; `circle` - Fuzzy controller  
&nbsp;  

## Functional block diagram:

![Functional block diagram](/Pictures/diagram8.png "Functional block diagram")

&nbsp;  

## Contact 
If you have further interest in SPHERE spherical robot, feel free to drop me an email at: <adrian.luberda96@gmail.com>

&nbsp;  

## Acknowledgements:

Thesis Supervisor and Originator:

**Adam Krzysztof Pilat Ph.D., D.Sc., Prof. AGH**  
<em>Active Levitation Technology - suspension and bearing  
Head of Robotic, Photovoltaic, and Magnetic Levitation Laboratory  
Faculty of Electrical Engineering, Automatics,  
Computer Science and Biomedical Engineering  
AGH - University of Science and Technology</em>

&nbsp;  

First version of the spherical robot created by:  

**Kacper Łanda, M.Sc.**  
<em>Faculty of Electrical Engineering, Automatics,  
Computer Science and Biomedical Engineering  
AGH - University of Science and Technology</em>