# whi_articulated_steering_controller
The kinematics controller for the articulated vehicle which consists of two separate components: a powered unit called the tractor or truck, and a trailer or semi-trailer that is attached to the tractor using a pivoting joint.

## Typical articulated kinematic model without slipping
![image](https://github.com/xinjuezou-whi/whi_articulated_steering_controller/assets/72239958/faf8f10a-b02f-4659-8740-5c47045a727f)

## Derivative of the tractor’s orientation angle
The **base_link** is set to the axis of the tractor's wheels, therefore the angular velocity for the tractor is defined as:

$$ \dot{\theta}_r = {linear * sin(\gamma) - l_f * \dot{\gamma} * cos(\gamma) \over l_f * cos(\gamma) + l_r} $$

Where $\theta_r$ denotes the orientation of the tractor, $l_r$ and $l_f$ represent the length of the tractor and the trailer to the pivot joint respectively, $\gamma$ is the position of the steer joint

## Parameters
Parameters $l_r$ and $l_f$ represent the length of the tractor and the trailer to the pivot joint respectively can be set through **wheel_separation_rear** and **wheel_separation_front**
