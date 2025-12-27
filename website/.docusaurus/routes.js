import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/book-hackathon/__docusaurus/debug',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug', '3a6'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/config',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/config', 'fd8'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/content',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/content', '018'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/globalData',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/globalData', 'fa6'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/metadata',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/metadata', '869'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/registry',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/registry', 'dec'),
    exact: true
  },
  {
    path: '/book-hackathon/__docusaurus/debug/routes',
    component: ComponentCreator('/book-hackathon/__docusaurus/debug/routes', '14d'),
    exact: true
  },
  {
    path: '/book-hackathon/docs',
    component: ComponentCreator('/book-hackathon/docs', '1fe'),
    routes: [
      {
        path: '/book-hackathon/docs',
        component: ComponentCreator('/book-hackathon/docs', 'f95'),
        routes: [
          {
            path: '/book-hackathon/docs',
            component: ComponentCreator('/book-hackathon/docs', 'c66'),
            routes: [
              {
                path: '/book-hackathon/docs/intro',
                component: ComponentCreator('/book-hackathon/docs/intro', 'a5a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/intro-ros2/',
                component: ComponentCreator('/book-hackathon/docs/intro-ros2/', 'e27'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/intro-ros2/concepts',
                component: ComponentCreator('/book-hackathon/docs/intro-ros2/concepts', 'df3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/intro-ros2/middleware',
                component: ComponentCreator('/book-hackathon/docs/intro-ros2/middleware', 'a2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/python-agents/',
                component: ComponentCreator('/book-hackathon/docs/python-agents/', 'cff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/python-agents/ai-integration',
                component: ComponentCreator('/book-hackathon/docs/python-agents/ai-integration', 'bf8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/python-agents/rclpy-basics',
                component: ComponentCreator('/book-hackathon/docs/python-agents/rclpy-basics', 'c91'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/urdf-structure/',
                component: ComponentCreator('/book-hackathon/docs/urdf-structure/', '955'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/urdf-structure/humanoid-models',
                component: ComponentCreator('/book-hackathon/docs/urdf-structure/humanoid-models', '491'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-hackathon/docs/urdf-structure/links-joints',
                component: ComponentCreator('/book-hackathon/docs/urdf-structure/links-joints', '3ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/book-hackathon/',
    component: ComponentCreator('/book-hackathon/', '436'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
