# whi_articulated_steering_controller
The kinematics controller for the articulated vehicle consists of two separate components: a powered unit called the tractor or truck, and a trailer or semi-trailer that is attached to the tractor using a pivoting joint.

## Typical articulated kinematic model without slipping
![image](https://github.com/xinjuezou-whi/whi_articulated_steering_controller/assets/72239958/faf8f10a-b02f-4659-8740-5c47045a727f)

## Derivative of the tractor’s orientation angle
The **base_link** is set to the axis of the tractor's wheels, therefore the angular velocity for the tractor is defined as:

$$ \dot{\theta}_r = {linear * sin(\gamma) - l_f * \dot{\gamma} * cos(\gamma) \over l_f * cos(\gamma) + l_r} $$

Where $\theta_r$ denotes the orientation of the tractor, $l_r$ and $l_f$ represent the length of the tractor and the trailer to the pivot joint respectively, $\gamma$ is the position of the steer joint

### Parameters
Parameters $l_r$ and $l_f$ represent the length of the tractor and the trailer to the pivot joint respectively can be set through **wheel_separation_rear** and **wheel_separation_front**

## Approximate solution for steering angle command
With the simplified non-slipping model, the velocity of vehicle's orientation is:

$$ \dot{\theta}_r = {linear * tan({\gamma \over 2}) \over L} $$

Where $L$ denotes the separation length of the tractor to the pivot joint and the trailer to the pivot joint since both separations are equal in the simplified model

Therefore the steering angle command can be deduced as:

$$ \gamma = 2 * atan({\dot{\theta}_r * L \over linear}) $$

## Dynamic footprint
Due to the tractor-trailer structure, the forms of the footprint change while the vehicle is turning. The fixed forms of the footprint will mislead the interference of occlusion. Fortunately, the costmap_2d subscribes to the footprint topic with geometry_msgs/Polygon message type, which provides a way for users to publish dynamically changing forms of the footprint
![image](https://github.com/xinjuezou-whi/whi_articulated_steering_controller/assets/72239958/02d10ad8-a8f1-438f-bfcf-80fcb5d63a37)

whi_articulated_steering_controller publishes the footprint with user configured topic:
![dynamic_footprint](https://github.com/xinjuezou-whi/whi_articulated_steering_controller/assets/72239958/5384b28a-0107-4849-ac4b-49dd65b5d30e)

### Parameters
In controller's parameter yaml file, set the following ones with specified values:
```
# Dynamic footprint
topic: footprint
trailer_top_left: [0.25, 0.08]
trailer_top_right: [0.25, -0.08]
tractor_bottom_left: [-0.08, 0.08]
tractor_bottom_right: [-0.08, -0.08]
```

In costmap's parameter yaml file, add the parameter "footprint_topic", if there is none, set its value aligned with the one in the controller's parameter file:
```
footprint_topic: /whi_01/NaviBOT/controller/base_controller/footprint
```

> NOTE: set the footprint_topic with an absolute path, which should include the full namespace, like above example
