import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Learn ROS 2 Fundamentals',
    description: (
      <>
        Understand the core concepts of ROS 2 including nodes, topics, services,
        and messages that form the foundation of robotic communication systems.
      </>
    ),
  },
  {
    title: 'Python Integration',
    description: (
      <>
        Master the integration of Python AI agents with ROS 2 using the rclpy library,
        bridging artificial intelligence with robotic control systems.
      </>
    ),
  },
  {
    title: 'Robot Modeling',
    description: (
      <>
        Explore URDF (Unified Robot Description Format) for modeling robot structures,
        including links, joints, and kinematics for humanoid robots.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}