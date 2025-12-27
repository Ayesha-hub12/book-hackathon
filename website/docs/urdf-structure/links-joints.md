---
sidebar_position: 2
---

# URDF Components: Links and Joints

## Understanding Robot Structure

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and their relationships.

### Links

A **link** represents a rigid body in the robot. Each link has:
- Physical properties (mass, inertia)
- Visual properties (shape, color, mesh)
- Collision properties (shape for collision detection)

**Basic link structure:**
```xml
<link name="link_name">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="1 1 1" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="1 1 1" />
    </geometry>
  </collision>
</link>
```

### Joints

A **joint** connects two links and defines how they can move relative to each other. Joints have types and limits:

**Types of joints:**
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis (with limits)
- **Continuous**: Rotational movement without limits
- **Prismatic**: Linear sliding movement (with limits)
- **Floating**: 6 DOF movement (for simulation)
- **Planar**: Movement in a plane

**Basic joint structure:**
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name" />
  <child link="child_link_name" />
  <origin xyz="1 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
</joint>
```

### Complete Simple Robot Example

Here's a simple robot with a base link and an arm:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link" />
    <child link="arm_link" />
    <origin xyz="0 0 0.15" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>
</robot>
```

### Kinematic Chains

Links and joints form kinematic chains that define how parts of the robot move relative to each other. The kinematic chain from base to end-effector is crucial for forward and inverse kinematics calculations.

### Visual and Collision Properties

- **Visual**: Defines how the link appears in visualizations
- **Collision**: Defines the shape used for collision detection
- These can be different shapes (e.g., detailed visual model, simplified collision model)

### Learning Objectives

After completing this section, you should be able to:
- Define links with physical and visual properties
- Create different types of joints with appropriate parameters
- Understand the relationship between links and joints
- Build simple kinematic chains in URDF