import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro-ros2">
            Read the Textbook - 15 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="A comprehensive guide to ROS 2 concepts for students with Python/AI basics">
      <HomepageHeader />
      <main>
        <section className={styles.about}>
          <div className="container padding-horiz--md">
            <h2>About This Book</h2>
            <p>
              This textbook provides a comprehensive guide to ROS 2 (Robot Operating System 2),
              designed specifically for students with Python and AI basics who want to learn about
              the middleware framework that powers modern robotics applications.
            </p>
            <p>
              The book is structured in three progressive modules that build upon each other:
            </p>
            <ul>
              <li><strong>Module 1:</strong> Introduction to ROS 2 concepts (nodes, topics, services, messages)</li>
              <li><strong>Module 2:</strong> Python Agents with ROS 2 integration using rclpy</li>
              <li><strong>Module 3:</strong> Robot Structure with URDF (Unified Robot Description Format)</li>
            </ul>
          </div>
        </section>
      </main>
    </Layout>
  );
}