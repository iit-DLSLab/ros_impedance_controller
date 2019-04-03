1) clone ros_impedance_controller into **dls_ws** 

2) git submodule update --init --recursive

3) hyqmake

4) roslaunch ros_impedance_controller ros_impedance_controller.launch

5) run /matlab_model_hyq/roscontroller.m to send joint position/velocity references **q_des**, **qd_des** and feed-forward torques **tau_ffwd**

