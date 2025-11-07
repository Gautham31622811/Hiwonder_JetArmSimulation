# Technical Specifications

## Robot Specifications

### Hiwonder JetArm

- **Degrees of Freedom (DOF)**: 6
- **Base Rotation**: ±180° (±3.14159 rad)
- **Joint Limits**:
  - Joint 1 (Base): -180° to +180°
  - Joint 2 (Shoulder): -90° to +90°
  - Joint 3 (Elbow): -90° to +90°
  - Joint 4 (Wrist Rotation): -90° to +90°
  - Joint 5 (Wrist Pitch): -90° to +90°
  - Joint 6 (Gripper): -45° to +45°

### Link Lengths

- Base Link: 60mm (height)
- Link 1: 60mm
- Link 2: 88mm
- Link 3: 88mm
- Link 4: 60mm
- Link 5: 60mm
- Gripper: 20mm

### Physical Properties

- **Total Reach**: ~356mm
- **Payload Capacity**: 100g
- **Operating Speed**: 1.0 rad/s (configurable)
- **Positioning Accuracy**: ±1cm

---

## Simulation Environment

### PyBullet Configuration

```python
Physics Engine: PyBullet 3.2.5+
Gravity: -9.81 m/s² (Z-axis)
Time Step: 1/240 seconds (default)
Solver Iterations: 50
Contact ERP: 0.2
Contact CFM: 0.0001
```

### World Coordinates

- **Origin**: Robot base center
- **X-Axis**: Forward (robot front)
- **Y-Axis**: Left (robot side)
- **Z-Axis**: Upward (vertical)

---

## Block Specifications

### Physical Properties

```python
Size: 30mm × 30mm × 30mm (cube)
Mass: 80g (spawned), 150g (gripped)
Colors: Red, Green, Blue (RGB)
Friction: 3.0 (lateral), 1.0 (spinning), 0.5 (rolling)
Restitution: 0.0 (no bounce)
```

### Color Definitions

- **Red**: RGB(1.0, 0.0, 0.0, 1.0)
- **Green**: RGB(0.0, 1.0, 0.0, 1.0)
- **Blue**: RGB(0.0, 0.0, 1.0, 1.0)

---

## Zone Configuration

### Zone Dimensions

All zones are rectangular:
- **Size**: 150mm × 150mm (15cm × 15cm)
- **Height**: 15mm above ground (1.5cm)
- **Separation**: 130mm between centers (13cm)

### Zone Positions

| Zone  | X (m) | Y (m)  | Z (m)  |
|-------|-------|--------|--------|
| Red   | 0.20  | 0.15   | 0.015  |
| Green | 0.20  | 0.02   | 0.015  |
| Blue  | 0.20  | -0.11  | 0.015  |

### Placement Heights

- **First Block (Base)**: 15mm (zone height)
- **Second Block (Stack)**: 45mm (15mm + 30mm block)

---

## Gripper System

### Constraint Parameters

```python
Type: JOINT_FIXED
Max Force: 500N
Damping: 0.5
Parent Frame Offset: [0, 0, -20mm]
```

### Grip Sequence

1. **Open Position**: -0.5 rad
2. **Close Position**: 0.6 rad
3. **Grip Force**: 20N
4. **Hold Duration**: 0.3s minimum

---

## Motion Control

### Movement Speeds

```python
Pickup Approach: 1.0s duration
Descent to Grasp: 0.6s duration
Lift with Block: 1.0s duration (slower for stability)
Base Rotation: 40 steps (0.015s per step)
Place Approach: 1.0s duration
Descent to Place: 1.2s duration
```

### Safety Margins

- **High Altitude**: 150mm above ground
- **Grasp Height**: Block height + 4mm
- **Safe Descent**: 50mm above target
- **Rotation Height**: 180mm (during base rotation)

---

## Performance Metrics

### Timing Benchmarks

- **Single Block Sort**: ~10-15 seconds
- **Complete Sequence (6 blocks)**: ~60-90 seconds
- **Constraint Establishment**: 0.3 seconds
- **Settling Time**: 0.6-0.8 seconds per placement

### Accuracy Targets

- **XY Positioning**: ±10mm (1cm tolerance)
- **Z Height**: ±5mm (0.5cm tolerance)
- **Zone Boundary**: 100% within rectangular bounds
- **Stack Alignment**: ±5mm offset

---

## Physics Parameters

### Block Dynamics (During Grip)

```python
Mass: 150g (increased for stability)
Lateral Friction: 5.0 (very high)
Linear Damping: 0.8 (reduces swing)
Angular Damping: 0.8 (reduces rotation)
```

### Constraint Settings

```python
Constraint Type: Fixed
Max Force: 500N
Error Reduction Parameter (ERP): 0.2
Constraint Force Mixing (CFM): 0.0001
```

---

## System Requirements

### Minimum Requirements

- **OS**: Windows 10/11, Ubuntu 18.04+, macOS 10.14+
- **CPU**: Intel i5 / AMD Ryzen 5 (4 cores)
- **RAM**: 4GB
- **GPU**: Integrated graphics (Intel HD 4000+)
- **Python**: 3.8+

### Recommended Requirements

- **CPU**: Intel i7 / AMD Ryzen 7 (8 cores)
- **RAM**: 8GB+
- **GPU**: Dedicated GPU (NVIDIA GTX 1050+)
- **Display**: 1920×1080 or higher

---

## Software Dependencies

```
pybullet >= 3.2.5
numpy >= 1.21.0
scipy >= 1.7.0
matplotlib >= 3.4.0 (optional, for visualization)
```

---

## Known Limitations

1. **Teleportation Required**: Final placement uses teleportation for precision
2. **Sequential Spawning**: Only one block can exist in pickup area at a time
3. **Fixed Zone Layout**: Zones are pre-configured, not dynamic
4. **Simplified Gripper**: Two-finger gripper simplified to rotation joint
5. **No Collision Detection**: Between zones (zones don't overlap)

---

## Future Improvements

- [ ] Eliminate teleportation with better IK solver
- [ ] Dynamic zone placement
- [ ] Multi-block simultaneous handling
- [ ] Computer vision for block detection
- [ ] Collision avoidance system
- [ ] Adaptive grip force based on block weight
- [ ] Real-time path planning

---

**Document Version**: 1.0  
**Last Updated**: November 2025  
**Maintained By**: Development Team
