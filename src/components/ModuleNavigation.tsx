import React, { useState } from 'react';
import Link from '@docusaurus/Link';

interface Chapter {
  title: string;
  path: string;
}

interface Module {
  id: number;
  title: string;
  icon: string;
  description: string;
  color: string;
  chapters: Chapter[];
}

const modulesData: Module[] = [
  {
    id: 1,
    title: 'ROS 2 Fundamentals',
    icon: '🤖',
    description: 'Build the robotic nervous system using ROS 2 nodes, topics, services, and URDF modeling',
    color: '#0066cc',
    chapters: [
      { title: 'Introduction to ROS 2', path: '/docs/module-01-ros2/introduction' },
      { title: 'Nodes and Topics', path: '/docs/module-01-ros2/nodes-topics' },
      { title: 'Services and Actions', path: '/docs/module-01-ros2/services-actions' },
      { title: 'URDF and Robot Modeling', path: '/docs/module-01-ros2/urdf-robot-modeling' },
      { title: 'Parameters and Launch Files', path: '/docs/module-01-ros2/parameters-launch' },
      { title: 'Mini Project: Basic Robot', path: '/docs/module-01-ros2/mini-project-basic-robot' },
    ],
  },
  {
    id: 2,
    title: 'Simulation & Digital Twins',
    icon: '🎮',
    description: 'Master Gazebo physics simulation and create safe virtual testing environments',
    color: '#00994d',
    chapters: [
      { title: 'Introduction to Simulation', path: '/docs/module-02-simulation/introduction' },
      { title: 'Gazebo Installation', path: '/docs/module-02-simulation/gazebo-installation' },
      { title: 'URDF/Xacro Models', path: '/docs/module-02-simulation/urdf-xacro-models' },
      { title: 'Sensor Simulation', path: '/docs/module-02-simulation/sensor-simulation' },
      { title: 'ROS 2 Gazebo Bridge', path: '/docs/module-02-simulation/ros2-gazebo-bridge' },
      { title: 'Unity Integration', path: '/docs/module-02-simulation/unity-integration' },
      { title: 'Mini Project: Walking Simulation', path: '/docs/module-02-simulation/mini-project-walking-simulation' },
    ],
  },
  {
    id: 3,
    title: 'Perception Systems',
    icon: '👁️',
    description: 'Enable robots to see and hear using computer vision and speech recognition',
    color: '#cc6600',
    chapters: [
      { title: 'Introduction to Perception', path: '/docs/module-03-perception/introduction' },
      { title: 'Computer Vision Basics', path: '/docs/module-03-perception/computer-vision-basics' },
      { title: 'Object Detection (YOLO)', path: '/docs/module-03-perception/object-detection' },
      { title: 'Speech Recognition (Whisper)', path: '/docs/module-03-perception/speech-recognition' },
      { title: 'Sensor Fusion', path: '/docs/module-03-perception/sensor-fusion' },
      { title: 'Mini Project: Voice Detection', path: '/docs/module-03-perception/mini-project-voice-detection' },
    ],
  },
  {
    id: 4,
    title: 'Planning & Control',
    icon: '🧠',
    description: 'Teach robots to plan tasks and execute motions safely with behavior trees',
    color: '#9900cc',
    chapters: [
      { title: 'Introduction to Control', path: '/docs/module-04-control/introduction' },
      { title: 'Motion Planning Basics', path: '/docs/module-04-control/motion-planning-basics' },
      { title: 'Behavior Trees (BT.CPP)', path: '/docs/module-04-control/behavior-trees' },
      { title: 'Task Planning', path: '/docs/module-04-control/task-planning' },
      { title: 'Trajectory Execution', path: '/docs/module-04-control/trajectory-execution' },
      { title: 'Mini Project: Task Planner', path: '/docs/module-04-control/mini-project-task-planner' },
    ],
  },
  {
    id: 5,
    title: 'Capstone Project',
    icon: '🏆',
    description: 'Integrate everything into a complete autonomous voice-controlled humanoid',
    color: '#cc0000',
    chapters: [
      { title: 'Introduction', path: '/docs/module-05-capstone/introduction' },
      { title: 'Integration Strategy', path: '/docs/module-05-capstone/integration-strategy' },
      { title: 'Workspace Setup', path: '/docs/module-05-capstone/workspace-setup' },
      { title: 'Demo and Deployment', path: '/docs/module-05-capstone/demo-deployment' },
    ],
  },
];

const ModuleNavigation: React.FC = () => {
  const [expandedModules, setExpandedModules] = useState<Set<number>>(new Set());

  const toggleModule = (moduleId: number) => {
    setExpandedModules((prev) => {
      const newSet = new Set(prev);
      if (newSet.has(moduleId)) {
        newSet.delete(moduleId);
      } else {
        newSet.add(moduleId);
      }
      return newSet;
    });
  };

  return (
    <div className="module-navigation">
      <h2 className="module-navigation-title">📚 Navigate by Module</h2>
      <p className="module-navigation-subtitle">
        Click on any module folder to explore its chapters
      </p>

      <div className="module-folders">
        {modulesData.map((module) => {
          const isExpanded = expandedModules.has(module.id);
          return (
            <div
              key={module.id}
              className={`module-folder ${isExpanded ? 'expanded' : 'collapsed'}`}
              style={{ borderLeftColor: module.color }}
            >
              <div
                className="module-folder-header"
                onClick={() => toggleModule(module.id)}
                role="button"
                tabIndex={0}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault();
                    toggleModule(module.id);
                  }
                }}
              >
                <span className="module-folder-icon">
                  {isExpanded ? '📂' : '📁'}
                </span>
                <div className="module-folder-title">
                  <span className="module-emoji">{module.icon}</span>
                  <span className="module-name">Module {module.id}: {module.title}</span>
                </div>
                <span className="module-toggle">
                  {isExpanded ? '▼' : '▶'}
                </span>
              </div>

              {isExpanded && (
                <div className="module-folder-content">
                  <p className="module-description">{module.description}</p>
                  <ul className="chapter-list">
                    {module.chapters.map((chapter, index) => (
                      <li key={index} className="chapter-item">
                        <Link to={chapter.path} className="chapter-link">
                          <span className="chapter-icon">📄</span>
                          <span className="chapter-title">{chapter.title}</span>
                        </Link>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default ModuleNavigation;
