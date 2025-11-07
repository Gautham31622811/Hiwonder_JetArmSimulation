
#!/usr/bin/env python3
"""
Hiwonder JetArm URDF/Gazebo Simulation - Objective 1: Pick and Place
=====================================================================
This scri      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.015 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.01 0.01"/>
      </geometry>
    </collision> <ma      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.023 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.046 0.02 0.02"/>
      </geometry>
    </collision>aterial>
    </visual>
    <collision>
      <origin xyz="0.03 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.025 0.025"/>
      </geometry>
    </collision>e="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.055 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.11 0.03 0.03"/>
      </geometry>
    </collision>e Hiwonder JetArm robotic arm in a URDF/Gazebo environment
with forward/inverse kinematics and pick-and-place capabilities.

Requirements:
    pip install pybullet numpy matplotlib scipy
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize


class JetArmURDFSimulation:
    """
    Hiwonder JetArm simulation in PyBullet (URDF-based physics engine)
    """
    
    def __init__(self, gui=True):
        """Initialize the simulation environment"""
        # Connect to PyBullet
        if gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Set up the simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        
        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Create or load JetArm URDF
        self.robot_id = self._create_jetarm_urdf()
        
        # Robot parameters (in meters and radians)
        self.link_lengths = [0.06, 0.088, 0.088, 0.06, 0.06]  # Link lengths
        self.num_joints = 6
        
        # Get joint information
        self.joint_indices = []
        self.joint_names = []
        self.joint_limits = []
        
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:  # Only revolute joints
                self.joint_indices.append(i)
                self.joint_names.append(joint_info[1].decode('utf-8'))
                self.joint_limits.append((joint_info[8], joint_info[9]))
        
        print(f"✓ Simulation initialized with {len(self.joint_indices)} joints")
        print(f"  Joint names: {self.joint_names}")
        
        # End-effector link index (usually the last link)
        self.end_effector_index = len(self.joint_indices) - 1
        
        # Object storage
        self.objects = []
        
    def _create_jetarm_urdf(self):
        """Create a URDF file for the Hiwonder JetArm if it doesn't exist"""
        urdf_path = "jetarm.urdf"
        
        if not os.path.exists(urdf_path):
            urdf_content = self._generate_jetarm_urdf()
            with open(urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"✓ Created URDF file: {urdf_path}")
        
        # Load the robot
        robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        return robot_id
    
    def _generate_jetarm_urdf(self):
        """Generate URDF XML content for the Hiwonder JetArm"""
        urdf = """<?xml version="1.0"?>
<robot name="hiwonder_jetarm">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.06"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1: Base Rotation -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.06"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57079" upper="1.57079" effort="10" velocity="1.0"/>
  </joint>

  <link name="link2">
    <visual>
      <origin xyz="0.044 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.088 0.03 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.044 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.088 0.03 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0.088 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57079" upper="1.57079" effort="10" velocity="1.0"/>
  </joint>

  <link name="link3">
    <visual>
      <origin xyz="0.044 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.088 0.025 0.025"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.044 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.088 0.025 0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 4: Wrist Rotation -->
  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0.088 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57079" upper="1.57079" effort="5" velocity="1.0"/>
  </joint>

  <link name="link4">
    <visual>
      <geometry>
        <cylinder radius="0.015" length="0.06"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.015" length="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Joint 5: Wrist Pitch -->
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57079" upper="1.57079" effort="5" velocity="1.0"/>
  </joint>

  <link name="link5">
    <visual>
      <origin xyz="0.03 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.02 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.03 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.08"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Joint 6: Gripper (simplified as revolute) -->
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="gripper"/>
    <origin xyz="0.06 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785398" upper="0.785398" effort="2" velocity="0.5"/>
  </joint>

  <link name="gripper">
    <visual>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

</robot>
"""
        return urdf
    
    def forward_kinematics(self, joint_angles):
        """
        Calculate end-effector position using forward kinematics
        
        Args:
            joint_angles: List/array of 6 joint angles (radians)
        
        Returns:
            End-effector position [x, y, z]
        """
        # Set joint positions
        for i, angle in enumerate(joint_angles[:len(self.joint_indices)]):
            p.resetJointState(self.robot_id, self.joint_indices[i], angle)
        
        # Get end-effector position
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        position = link_state[4]  # World position
        orientation = link_state[5]  # World orientation (quaternion)
        
        return np.array(position), np.array(orientation)
    
    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Calculate joint angles to reach target position using IK
        
        Args:
            target_position: Target [x, y, z] position
            target_orientation: Optional target orientation (quaternion)
        
        Returns:
            Joint angles array
        """
        if target_orientation is None:
            # Default orientation (pointing down)
            target_orientation = p.getQuaternionFromEuler([0, np.pi/2, 0])
        
        # Use PyBullet's built-in IK solver
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_index,
            target_position,
            target_orientation,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return np.array(joint_poses[:len(self.joint_indices)])
    
    def move_to_joint_angles(self, joint_angles, duration=2.0, steps=100):
        """
        Smoothly move robot to target joint angles
        
        Args:
            joint_angles: Target joint angles
            duration: Time to complete movement (seconds)
            steps: Number of interpolation steps
        """
        # Get current joint angles
        current_angles = []
        for joint_idx in self.joint_indices:
            joint_state = p.getJointState(self.robot_id, joint_idx)
            current_angles.append(joint_state[0])
        current_angles = np.array(current_angles)
        
        # Interpolate between current and target
        for step in range(steps):
            alpha = step / steps
            interpolated_angles = current_angles + alpha * (joint_angles - current_angles)
            
            # Set joint positions with control
            for i, joint_idx in enumerate(self.joint_indices):
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=interpolated_angles[i],
                    force=50
                )
            
            p.stepSimulation()
            time.sleep(duration / steps)
    
    def move_to_position(self, target_position, target_orientation=None, duration=2.0):
        """
        Move end-effector to target position using IK
        
        Args:
            target_position: Target [x, y, z] position
            target_orientation: Optional target orientation
            duration: Time to complete movement
        """
        joint_angles = self.inverse_kinematics(target_position, target_orientation)
        self.move_to_joint_angles(joint_angles, duration)
        return joint_angles
    
    def spawn_object(self, position, color=[1, 0, 0, 1], size=0.03):
        """
        Spawn a cube object in the simulation
        
        Args:
            position: [x, y, z] position
            color: RGBA color
            size: Size of the cube
        
        Returns:
            Object ID
        """
        # Create visual and collision shapes
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2],
            rgbaColor=color
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2]
        )
        
        # Create multi-body
        obj_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        self.objects.append(obj_id)
        return obj_id
    
    def pick_object(self, object_position, approach_height=0.15, grasp_height=None):
        """
        Pick up an object at the specified position
        
        Args:
            object_position: [x, y, z] position of object
            approach_height: Height to approach from above
            grasp_height: Height to grasp at (defaults to object z)
        """
        if grasp_height is None:
            grasp_height = object_position[2]
        
        print(f"→ Picking object at {object_position}")
        
        # 1. Move above object
        approach_pos = [object_position[0], object_position[1], approach_height]
        print("  Step 1: Moving to approach position...")
        self.move_to_position(approach_pos, duration=1.5)
        time.sleep(0.5)
        
        # 2. Move down to object
        grasp_pos = [object_position[0], object_position[1], grasp_height]
        print("  Step 2: Moving down to grasp...")
        self.move_to_position(grasp_pos, duration=1.0)
        time.sleep(0.5)
        
        # 3. Close gripper (simplified - just set gripper joint)
        print("  Step 3: Closing gripper...")
        if len(self.joint_indices) > 5:
            p.setJointMotorControl2(
                self.robot_id,
                self.joint_indices[5],
                p.POSITION_CONTROL,
                targetPosition=0.5,
                force=10
            )
        time.sleep(0.5)
        
        # 4. Lift object
        lift_pos = [object_position[0], object_position[1], approach_height]
        print("  Step 4: Lifting object...")
        self.move_to_position(lift_pos, duration=1.0)
        time.sleep(0.5)
        
        print("✓ Pick complete")
    
    def place_object(self, target_position, approach_height=0.15):
        """
        Place object at target position
        
        Args:
            target_position: [x, y, z] position to place object
            approach_height: Height to approach from above
        """
        print(f"→ Placing object at {target_position}")
        
        # 1. Move above target
        approach_pos = [target_position[0], target_position[1], approach_height]
        print("  Step 1: Moving to approach position...")
        self.move_to_position(approach_pos, duration=1.5)
        time.sleep(0.5)
        
        # 2. Move down to place
        print("  Step 2: Moving down to place...")
        self.move_to_position(target_position, duration=1.0)
        time.sleep(0.5)
        
        # 3. Open gripper
        print("  Step 3: Opening gripper...")
        if len(self.joint_indices) > 5:
            p.setJointMotorControl2(
                self.robot_id,
                self.joint_indices[5],
                p.POSITION_CONTROL,
                targetPosition=-0.5,
                force=10
            )
        time.sleep(0.5)
        
        # 4. Retract
        retract_pos = [target_position[0], target_position[1], approach_height]
        print("  Step 4: Retracting...")
        self.move_to_position(retract_pos, duration=1.0)
        time.sleep(0.5)
        
        print("✓ Place complete")
    
    def pick_and_place(self, pick_pos, place_pos):
        """
        Complete pick and place operation
        
        Args:
            pick_pos: Position to pick from [x, y, z]
            place_pos: Position to place at [x, y, z]
        """
        print("\n" + "="*60)
        print("PICK AND PLACE OPERATION")
        print("="*60)
        
        self.pick_object(pick_pos)
        self.place_object(place_pos)
        
        print("="*60)
        print("✓ Pick and place operation complete!")
        print("="*60 + "\n")
    
    def go_to_home_position(self):
        """Move robot to home/rest position"""
        home_angles = [0, 0, -np.pi/4, 0, np.pi/4, 0]
        print("→ Moving to home position...")
        self.move_to_joint_angles(home_angles, duration=2.0)
        time.sleep(0.5)
        print("✓ At home position")
    
    def close(self):
        """Close the simulation"""
        p.disconnect()


def demo_objective1_pick_and_place():
    """
    Demonstration of Objective 1: Pick and Place with URDF simulation
    """
    print("\n" + "="*70)
    print(" HIWONDER JETARM - OBJECTIVE 1: PICK AND PLACE SIMULATION")
    print(" URDF/Gazebo Environment (PyBullet)")
    print("="*70 + "\n")
    
    # Initialize simulation
    sim = JetArmURDFSimulation(gui=True)
    
    # Move to home position
    sim.go_to_home_position()
    
    # Spawn objects at different positions
    print("\n→ Spawning objects...")
    obj1 = sim.spawn_object([0.15, 0.10, 0.03], color=[1, 0, 0, 1])  # Red
    obj2 = sim.spawn_object([0.15, -0.10, 0.03], color=[0, 1, 0, 1])  # Green
    obj3 = sim.spawn_object([0.20, 0.00, 0.03], color=[0, 0, 1, 1])  # Blue
    print(f"✓ Spawned {len(sim.objects)} objects\n")
    
    time.sleep(1)
    
    # Demonstration 1: Pick and place object 1
    sim.pick_and_place(
        pick_pos=[0.15, 0.10, 0.03],
        place_pos=[-0.10, 0.15, 0.03]
    )
    time.sleep(1)
    
    # Demonstration 2: Pick and place object 2
    sim.pick_and_place(
        pick_pos=[0.15, -0.10, 0.03],
        place_pos=[-0.10, 0.00, 0.03]
    )
    time.sleep(1)
    
    # Demonstration 3: Pick and place object 3
    sim.pick_and_place(
        pick_pos=[0.20, 0.00, 0.03],
        place_pos=[-0.10, -0.15, 0.03]
    )
    time.sleep(1)
    
    # Return to home
    sim.go_to_home_position()
    
    print("\n" + "="*70)
    print(" DEMONSTRATION COMPLETE!")
    print(" Press Ctrl+C or close window to exit")
    print("="*70 + "\n")
    
    # Keep simulation running
    try:
        while True:
            p.stepSimulation()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n→ Closing simulation...")
        sim.close()
        print("✓ Simulation closed")


def test_kinematics():
    """Test forward and inverse kinematics"""
    print("\n" + "="*70)
    print(" KINEMATICS TEST")
    print("="*70 + "\n")
    
    sim = JetArmURDFSimulation(gui=True)
    
    # Test forward kinematics
    print("→ Testing Forward Kinematics...")
    test_angles = [0, np.pi/4, -np.pi/4, 0, 0, 0]
    position, orientation = sim.forward_kinematics(test_angles)
    print(f"  Joint angles: {np.round(test_angles, 3)}")
    print(f"  End-effector position: {np.round(position, 4)}")
    print(f"  End-effector orientation: {np.round(orientation, 4)}\n")
    
    # Test inverse kinematics
    print("→ Testing Inverse Kinematics...")
    target_pos = [0.15, 0.10, 0.15]
    calculated_angles = sim.inverse_kinematics(target_pos)
    print(f"  Target position: {target_pos}")
    print(f"  Calculated joint angles: {np.round(calculated_angles, 3)}")
    
    # Verify by moving there
    sim.move_to_position(target_pos, duration=2.0)
    actual_pos, _ = sim.forward_kinematics(calculated_angles)
    print(f"  Actual position reached: {np.round(actual_pos, 4)}")
    error = np.linalg.norm(np.array(target_pos) - actual_pos)
    print(f"  Position error: {error:.6f} m\n")
    
    time.sleep(2)
    sim.close()
    print("✓ Kinematics test complete")


if __name__ == "__main__":
    import sys
    
    print("\n" + "="*70)
    print(" HIWONDER JETARM URDF SIMULATION")
    print("="*70)
    print("\nSelect mode:")
    print("  1. Full Pick and Place Demonstration (Objective 1)")
    print("  2. Kinematics Test")
    print("  3. Exit")
    
    choice = input("\nEnter choice (1-3): ").strip()
    
    if choice == "1":
        demo_objective1_pick_and_place()
    elif choice == "2":
        test_kinematics()
    elif choice == "3":
        print("Exiting...")
        sys.exit(0)
    else:
        print("Invalid choice. Running full demonstration...")
        demo_objective1_pick_and_place()
