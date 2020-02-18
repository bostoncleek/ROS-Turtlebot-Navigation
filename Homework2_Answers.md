## Boston Cleek

# TODO: MERGE TO MASTER
# TODO: finish answering questions and take video file /briefs and func docs



# Physical Testing

Observations: The use of encoders improves the estimate of the robot's odometry because the current model does not include wheel slip. Moving slower tends to improve the estimate of the pose. This is likely a result of less wheel slip and the ability of the encoders the capture the wheels orientation at lower speeds.


## 1)

### Rotation
frac_vel = 1
AV = 2.84

#### Clockwise rotation
ET = 0
EX = 0
EY = 0

OT = 121
OX = 0
OY = 0

FT = 24
FX = 0
FY = 0   

GT = 135
GX = 0
GY = 0

DT = 0.7
DX = 0
DY = 0


#### Counter-Clockwise rotation
ET = 0
EX = 0
EY = 0

OT = 173
OX = 0
OY = 0

FT = 26
FX = 0
FY = 0

GT = 135
GX = 0
GY = 0

DT = 1.9
DX = 0
DY = 0

### Translation
frac_vel = 1
AV = 0.22

#### Forward
ET = 0
EX = 2
EY = 0

OT = 1.3
OX = 1.83
OY = 0

FT = 0
FX = 2.1
FY = 0   

GT = 10
GX = 2.03
GY = 0

DT = 2.03
DX = 0
DY = 0


#### Backward
ET = 0
EX = 2
EY = 0

OT = 19
OX = 1.8
OY = 0.4

FT = 0
FX = 2.013
FY = 0

GT = 19
GX = 1.81
GY = 0.27

DT = 0.9
DX = 0.001
DY = 0.013



## 2)
frac_vel = 0.5
AV = 1.42


### Clockwise rotation
ET = 0
EX = 0
EY = 0

OT = 50
OX = 0
OY = 0

FT = 21
FX = 0
FY = 0   

GT = 0
GX = 0
GY = 0

DT = 2.5
DX = 0
DY = 0

### Counter-Clockwise rotation
ET = 0
EX = 0
EY = 0

OT = 41
OX = 0
OY = 0

FT = 24
FX = 0
FY = 0

GT = 10
GX = 0
GY = 0

DT = 1.55
DX = 0
DY = 0

### Translation
frac_vel = 0.55
AV = 0.11

#### Forward
ET = 0
EX = 2
EY = 0

OT = 0
OX = 2.015
OY = 0

FT = 0
FX = 1.995
FY = 0   

GT = 0
GX = 2.032
GY = 0

DT = 0
DX = 0.0017
DY = 0


#### Backward
ET = 0
EX = 2
EY = 0

OT = 0
OX = 2.026
OY = 0

FT = 0
FX = 1.991
FY = 0

GT = 0
GX = 2.032
GY = 0

DT = 0
DX = 0.0006
DY = 0

## 3) Waypoint Following
After 1 cycle using frac_vel = 0.5 the robot is y error: 0.12m and x error: 0.10m from the start and the total distance traveled if 5.23m.
