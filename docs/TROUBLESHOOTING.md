# Troubleshooting Guide

## Common Issues and Solutions

---

## 1. Block Slipping from Gripper

### Symptom
- Block is picked up successfully
- During rotation or movement, block slides backward
- Block drops before reaching target zone

### Root Causes
1. **Weak constraint force** - Default 100N may be insufficient
2. **Incorrect gripper link index** - Constraint attached to wrong link
3. **Fast movements** - Acceleration exceeds grip strength
4. **Low physics damping** - Block swings/spins during transport

### Solutions

#### Solution A: Verify Gripper Link Index (MOST COMMON)

```python
# Check console for debug output:
# "→ DEBUG: Creating constraint with gripper link X"

# If link index looks wrong, verify:
num_joints = p.getNumJoints(robot_id)
gripper_link = num_joints - 1  # Should be 6 for JetArm

# DON'T use: len(joint_indices) - 1
# This may give wrong link if indices are not sequential
```

**Expected Output**: `Creating constraint with gripper link 6`

---

#### Solution B: Increase Constraint Force

```python
# In optimized_sequential_sorting.py, line ~540
self.constraint_id = p.createConstraint(...)
p.changeConstraint(self.constraint_id, maxForce=500)  # Increase from 100

# If still slipping, try:
p.changeConstraint(self.constraint_id, maxForce=1000)  # Very strong
```

**Recommended**: 500N for most cases, 1000N for extreme reliability

---

#### Solution C: Add Physics Damping

```python
# After creating constraint, increase block damping
p.changeDynamics(
    object_id,
    -1,
    mass=0.15,              # Heavier = more stable
    lateralFriction=5.0,    # Higher friction
    linearDamping=0.8,      # Reduces swing
    angularDamping=0.8      # Reduces spin
)
```

---

#### Solution D: Slow Down Movements

```python
# Increase movement duration in smooth_move_to()
self.smooth_move_to(target_pos, duration=1.0)  # Increase from 0.6s

# Increase rotation steps
for step in range(40):  # Increase from 20
    angle = ...
    p.resetJointState(...)
```

---

#### Solution E: Check Constraint Attachment

```python
# After creating constraint, verify it's valid
if self.constraint_id is not None:
    # Get constraint info
    info = p.getConstraintInfo(self.constraint_id)
    print(f"Constraint attached to link: {info[2]}")
    
    # Verify block position remains stable
    block_pos_before = p.getBasePositionAndOrientation(object_id)[0]
    for _ in range(30):
        p.stepSimulation()
    block_pos_after = p.getBasePositionAndOrientation(object_id)[0]
    
    distance = np.linalg.norm(np.array(block_pos_after) - np.array(block_pos_before))
    if distance > 0.01:
        print(f"⚠️ WARNING: Block moved {distance:.3f}m after constraint!")
```

---

## 2. Blue Blocks Not Picked Up

### Symptom
- Arm approaches blue blocks but gripper misses
- Gripper closes on empty space
- Blue blocks untouched while red/green blocks work

### Root Cause
- Blue blocks at negative Y positions (-0.18, -0.24)
- Arm base at 0° rotation (facing +Y direction)
- Gripper cannot reach negative Y without base rotation

### Solution

```python
# Calculate base angle to face block
import math

def calculate_pickup_angle(pick_pos):
    return math.atan2(pick_pos[1], pick_pos[0])

# Rotate base before pickup
pickup_angle = calculate_pickup_angle(pick_pos)
current_base_angle = p.getJointState(self.sim.robot_id, 0)[0]

if abs(current_base_angle - pickup_angle) > 0.05:
    # Rotate to pickup position
    steps = 20
    for i in range(steps + 1):
        angle = current_base_angle + (pickup_angle - current_base_angle) * (i / steps)
        p.resetJointState(self.sim.robot_id, 0, angle)
        for _ in range(2):
            p.stepSimulation()
```

**Result**: Arm rotates to face blue blocks before approaching

---

## 3. Blocks Spawning Outside Pickup Area

### Symptom
- Blocks appear at random locations
- Blocks fall off the floor
- Pickup positions don't match expected locations

### Root Cause
- Pickup grid coordinates incorrect
- Floor size too small
- Physics not stabilized after spawn

### Solution

```python
# Use verified pickup grid
pickup_grid = {
    0: [0.20, -0.24, 0.015],  # Block 1: RED (rear left)
    1: [0.20, -0.21, 0.015],  # Block 2: GREEN (middle left)
    2: [0.20, -0.18, 0.015],  # Block 3: BLUE (front left)
    3: [0.18, -0.24, 0.015],  # Block 4: RED (rear right)
    4: [0.18, -0.21, 0.015],  # Block 5: GREEN (middle right)
    5: [0.18, -0.18, 0.015]   # Block 6: BLUE (front right)
}

# Ensure floor is large enough
floor_size = [0.8, 0.8, 0.01]  # 80cm x 80cm

# Add settling time after spawn
p.loadURDF(..., basePosition=spawn_pos)
for _ in range(60):  # 0.6 seconds
    p.stepSimulation()
```

---

## 4. Blocks Falling Through Floor

### Symptom
- Blocks disappear after spawning
- Blocks at Z < 0 position
- Console shows large negative Z velocities

### Root Cause
- Collision disabled between block and floor
- Floor collision shape incorrect
- Block spawned below floor level

### Solution

```python
# Enable collision flags
block_id = p.loadURDF(
    urdf_path,
    basePosition=spawn_pos,
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
)

# Verify floor has collision
floor_info = p.getCollisionShapeData(floor_id, -1)
print(f"Floor collision type: {floor_info}")

# Spawn blocks slightly above floor
spawn_pos = [x, y, 0.015]  # 1.5cm above ground (half block height)
```

---

## 5. Arm Movements Too Fast/Jerky

### Symptom
- Robot arm moves in sudden jumps
- Blocks shake during transport
- Arm overshoots target positions

### Root Cause
- Movement duration too short
- Not enough interpolation steps
- No velocity smoothing

### Solution

```python
def smooth_move_to(self, target_pos, duration=1.0):
    """Increase duration for smoother movement"""
    start_time = time.time()
    start_pos = self.get_end_effector_position()
    
    while True:
        elapsed = time.time() - start_time
        t = min(elapsed / duration, 1.0)  # 0 to 1
        
        # Smooth interpolation
        current_pos = [
            start_pos[i] + (target_pos[i] - start_pos[i]) * t
            for i in range(3)
        ]
        
        # Solve IK and update
        joint_angles = p.calculateInverseKinematics(...)
        for idx, angle in zip(self.sim.joint_indices, joint_angles):
            p.setJointMotorControl2(
                self.sim.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=100,
                maxVelocity=0.5  # Limit max velocity
            )
        
        p.stepSimulation()
        
        if t >= 1.0:
            break
```

**Recommended**: 
- Pickup/place movements: 1.0s duration
- Rotation movements: 40 steps minimum
- Max velocity: 0.5 rad/s

---

## 6. Zone Placement Inaccurate

### Symptom
- Blocks land outside zone boundaries
- Blocks placed at wrong heights
- Second blocks miss the first block

### Root Cause
- Zone center coordinates incorrect
- Teleport position not calculated properly
- Stack height not accounting for block size

### Solution

```python
# Verified zone centers
zone_centers = {
    'red': [0.20, 0.15, 0.015],    # Right zone
    'green': [0.20, 0.02, 0.015],  # Center zone
    'blue': [0.20, -0.11, 0.015]   # Left zone
}

# Correct stack height calculation
block_size = 0.03  # 3cm cube
stack_height = zone_z + block_size + 0.015  # zone + block + half-block

# Verify placement
def verify_in_zone(block_pos, zone_center, zone_size):
    x_ok = abs(block_pos[0] - zone_center[0]) <= zone_size[0] / 2
    y_ok = abs(block_pos[1] - zone_center[1]) <= zone_size[1] / 2
    return x_ok and y_ok
```

---

## 7. Constraint Creation Fails (None)

### Symptom
- Console shows `self.constraint_id = None`
- Block drops immediately after "pickup"
- Error: "Constraint not created"

### Root Cause
- Invalid gripper link index
- Invalid block ID
- Robot or block not fully loaded

### Solution

```python
# Verify both bodies exist
if robot_id < 0 or object_id < 0:
    print("ERROR: Invalid body ID")
    return

# Get correct gripper link
num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints")

# Last link should be gripper
gripper_link = num_joints - 1

# Create constraint with debug
self.constraint_id = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=gripper_link,
    childBodyUniqueId=object_id,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, -0.02],
    childFramePosition=[0, 0, 0]
)

print(f"Constraint ID: {self.constraint_id}")

if self.constraint_id is not None:
    p.changeConstraint(self.constraint_id, maxForce=500)
    print("✓ Constraint created successfully")
else:
    print("✗ Constraint creation FAILED")
```

---

## 8. ImportError: No module named 'pybullet'

### Symptom
```
ImportError: No module named 'pybullet'
```

### Solution

```bash
# Install PyBullet
pip install pybullet

# Or install all requirements
pip install -r requirements.txt

# Verify installation
python -c "import pybullet as p; print(p.getVersionInfo())"
```

**Expected Output**: `(3, 2, 5)` or higher

---

## 9. GUI Window Not Appearing

### Symptom
- Script runs but no visualization
- Black screen or frozen window

### Solution

```python
# Ensure GUI mode enabled
p.connect(p.GUI)  # NOT p.DIRECT

# Check graphics card drivers
# Update: NVIDIA/AMD drivers

# Alternative: Use DIRECT mode for headless
p.connect(p.DIRECT)  # No GUI, faster simulation
```

---

## 10. Performance Issues (Low FPS)

### Symptom
- Simulation runs very slowly
- <30 FPS rendering
- Long delays between actions

### Solutions

```python
# A. Reduce physics timestep
p.setTimeStep(0.01)  # Increase from 0.001

# B. Reduce settling time
for _ in range(30):  # Reduce from 60
    p.stepSimulation()

# C. Disable rendering during simulation
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# ... run simulation ...
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# D. Use DIRECT mode (no GUI)
p.connect(p.DIRECT)
```

---

## Debugging Checklist

### Before Running Simulation

- [ ] Python 3.8+ installed
- [ ] PyBullet 3.2.5+ installed (`pip list | grep pybullet`)
- [ ] `jetarm.urdf` exists in `assets/` folder
- [ ] All imports successful (no ImportError)

### During Simulation

- [ ] Check console for debug messages
- [ ] Verify constraint ID is not None
- [ ] Verify gripper link index (should be 6)
- [ ] Check block positions after spawn
- [ ] Monitor block position during transport

### After Simulation

- [ ] All 6 blocks sorted correctly
- [ ] Each block in correct color zone
- [ ] Second blocks stacked on first blocks
- [ ] No blocks outside zone boundaries

---

## Advanced Debugging

### Enable Detailed Logging

```python
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# In code
logging.debug(f"Block {i} spawned at {spawn_pos}")
logging.debug(f"Constraint ID: {self.constraint_id}")
logging.debug(f"Gripper link: {gripper_link}")
logging.debug(f"Block final pos: {final_pos}")
```

### Visualize Coordinate Frames

```python
# Show world frame
p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0])  # X-axis red
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0])  # Y-axis green
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1])  # Z-axis blue

# Show zone boundaries
def draw_zone_boundary(center, size):
    corners = [
        [center[0] - size[0]/2, center[1] - size[1]/2, center[2]],
        [center[0] + size[0]/2, center[1] - size[1]/2, center[2]],
        [center[0] + size[0]/2, center[1] + size[1]/2, center[2]],
        [center[0] - size[0]/2, center[1] + size[1]/2, center[2]]
    ]
    for i in range(4):
        p.addUserDebugLine(corners[i], corners[(i+1)%4], [1, 1, 0])
```

---

## Getting Help

If issues persist:

1. **Check debug output**: Look for "DEBUG:", "ERROR:", "WARNING:" messages
2. **Run with verbose logging**: Add `logging.DEBUG` mode
3. **Verify physics parameters**: Print constraint force, block mass, friction
4. **Test incrementally**: Comment out placement, test pickup only
5. **Share console output**: Copy full terminal output for diagnosis

**Contact**: Include your console output, `main.py` modifications, and Python/PyBullet versions.

---

**Last Updated**: November 2025  
**Applies to**: JetArm Color Sorting Challenge v2.0
