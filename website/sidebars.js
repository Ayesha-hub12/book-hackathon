// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to ROS 2',
          items: [
            'intro-ros2/index',
            'intro-ros2/concepts',
            'intro-ros2/middleware'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Python Agents with ROS 2',
          items: [
            'python-agents/index',
            'python-agents/rclpy-basics',
            'python-agents/ai-integration'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Robot Structure with URDF',
          items: [
            'urdf-structure/index',
            'urdf-structure/links-joints',
            'urdf-structure/humanoid-models'
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;