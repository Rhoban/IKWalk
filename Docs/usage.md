# Walk Eigine Usage

First include de header file:
```cpp
#include <IKWalk.hpp>
```

Parameters have to be initialized with
both humanoid model leg length and generator parameters
(See [all available parameters](parameters.md))
```cpp
struct Rhoban::IKWalkParameters params;
//Model
params.distHipToKnee = 0.093;
params.distKneeToAnkle = 0.105;
params.distAnkleToGround = 0.032;
params.distFeetLateral = 0.092;
//Generator
params.freq = 1.7;
...
```
Leg distance between rotation axis are given in meters.

The walk generator is then called by a static method and both
```phase``` and ```outputs``` are updated:
```cpp
double dt = 0.02;
double phase = 0.0;
while (...) {
    struct Rhoban::IKWalkOutputs outputs;
    bool success = Rhoban::IKWalk::walk(
        params, dt, phase, outputs);
    ...
}
```
```phase``` is the engine state between 0 and 1 representing the current position
in the walking cycle. The variable is updated according to walk frequency and
the time step ```dt``` (in seconds). 
The walk engine returns ```false``` if given parameters lead to unreachable position
for the inverse kinematics and ```phase``` is not updated.

Motor target positions are available in ```Rhoban::IKWalkOutputs``` structure (in radians)
```cpp
outputs.left_hip_yaw
outputs.left_hip_pitch
outputs.left_hip_roll
outputs.left_knee
outputs.left_ankle_pitch
outputs.left_ankle_roll
outputs.right_hip_yaw
outputs.right_hip_pitch
outputs.right_hip_roll
outputs.right_knee
outputs.right_ankle_pitch
outputs.right_ankle_roll
```

The omnidirectional walking is achieved by mixing by these four parameters:
* ```params.stepGain```: Forward (X) footstep length (in meters)
* ```params.lateralGain```: Lateral (Y) footstep length (in meters)
* ```params.turnGain```: Footstep rotation angle (in radian)
* ```params.enabledGain```: Additional gain multiplying all oscillatory movements. The walk is stopped if set to 0 and fully enabled if set to 1.

