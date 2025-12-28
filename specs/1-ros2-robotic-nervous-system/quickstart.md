# Quickstart Guide: ROS 2 Textbook Module

**Feature**: 1-ros2-robotic-nervous-system
**Date**: 2025-12-27

## Getting Started

This guide will help you set up and run the ROS 2 Textbook Module locally.

### Prerequisites

- Node.js 18 or higher
- npm or yarn package manager
- Git for version control

### Installation

1. Clone the repository:
```bash
git clone [repository-url]
cd [repository-name]
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

This will start a local development server at `http://localhost:3000` with live reloading.

### Project Structure

The textbook content is organized in the `docs/` directory:

```
docs/
├── intro-ros2/         # Chapter 1: Introduction to ROS 2
│   ├── index.md
│   ├── concepts.md
│   └── middleware.md
├── python-agents/      # Chapter 2: Python Agents with ROS 2
│   ├── index.md
│   ├── rclpy-basics.md
│   └── ai-integration.md
└── urdf-structure/     # Chapter 3: Robot Structure with URDF
    ├── index.md
    ├── links-joints.md
    └── humanoid-models.md
```

### Adding New Content

1. Create a new markdown file in the appropriate chapter directory
2. Add the file to the sidebar configuration in `sidebars.js`
3. Use the frontmatter to specify the title and other metadata:

```markdown
---
title: Your Content Title
sidebar_position: 1
---

Your content here...
```

### Building for Production

To build the static site for deployment:

```bash
npm run build
```

The output will be in the `build/` directory and can be deployed to any static hosting service.

### Local Preview

To preview the production build locally:

```bash
npm run serve
```

### Configuration

The main configuration is in `docusaurus.config.js` where you can:
- Update site metadata (title, description)
- Configure navigation
- Set up analytics
- Customize styling