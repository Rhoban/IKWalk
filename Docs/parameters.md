# Walk Engine Parameters

The structure ```Rhoban::IKWalkParameters``` has following parameters:

* ```distHipToKnee```: Leg model distance between hip and knee rotation axes (in meters).
* ```distKneeToAnkle```: Leg model distance between knee and ankle rotation axes (in meters).
* ```distAnkleToGround```: Leg model distance between ankle rotation axis and the foot center (in meters).
* ```distFeetLateral```: Leg model lateral distance between the two feet center (in meters).

* ```enabledGain```: Allows for smoothing the walk stopping and starting. =0 stops the walk. =1 starts the walk.
* ```stepGain```: Footstep forward (X) length (in meters). <0 goes backward. Note that you may have to find the actual value offset for the robot neutral.
* ```lateralGain```: Footstep lateral (Y) length (in meters). =0 goes straight. >0 makes lateral steps on the left. <0 makes lateral steps on the right.
* ```turnGain```: Footstep rotation angle (in radians). =0 goes straight. >0 turns on the left. <0 turns steps on the right.

* ```freq```: Complete walk cycle (two steps) frequency (in Hertz).
* ```supportPhaseRatio```: The length of the double support phase in walk cycle. =0 is full single support, no double support phase. =1 is full double support phase.
* ```footYOffset```: Lateral distance offset between the two foot (in meters).
* ```riseGain```: Foot height (Z) during flying backward to forward (in meters).
* ```swingGain```: Amplitude of lateral oscillation of the trunk with respect to the feet (in meters).
* ```swingPhase```: Phase shift used to desynchronize the lateral trunk oscillation and footsteps movement.

* ```trunkXOffset```: Trunk forward (X) translation with respect to the feet (in meters). >0 goes forward. <0 goes backward.
* ```trunkYOffset```: Trunk lateral (Y) translation with respect to the feet (in meters). >0 goes on the left. <0 goes on the right.
* ```trunkZOffset```: Trunk height from the ground. =0 is maximum height (in meters). >0 is lower to the ground.
* ```trunkPitch```: Trunk pitch (Y) orientation. >0 bends forward (in radians). <0 bends backward.
* ```trunkRoll```: Trunk roll (X) orientation. >0 bends on the left (in radians). <0 bends on the right.

* ```swingPause```: 
* ```swingVel```: 
* ```stepUpVel```: 
* ```stepDownVel```: 
* ```riseUpVel```: 
* ```riseDownVel```: 

    
Additional offset on left and right foot target position (in meters). They must be set to zero if not used.
* ```extraLeftX```
* ```extraLeftY```
* ```extraLeftZ```
* ```extraRightX```
* ```extraRightY```
* ```extraRightZ```
Additional offset on left and right foot target orientation (in radians). They must be set to zero if not used.
* ```extraLeftYaw```
* ```extraLeftPitch```
* ```extraLeftRoll```
* ```extraRightYaw```
* ```extraRightPitch```
* ```extraRightRoll```

The ```extra``` offsets can for example be used to implement a dynamic kick on top of the walk engine. 

