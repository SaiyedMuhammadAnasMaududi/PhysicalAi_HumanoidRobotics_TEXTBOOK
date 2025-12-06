import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: ['intro'],

  module1Sidebar: [
    'module-01-ros2/introduction',
    'module-01-ros2/ros2-installation-setup',
    'module-01-ros2/nodes-topics-services',
    'module-01-ros2/publishers-subscribers',
    'module-01-ros2/humanoid-urdf',
    'module-01-ros2/mini-project-ros2-communication',
  ],

  module2Sidebar: [
    'module-02-simulation/introduction',
    'module-02-simulation/gazebo-installation',
    'module-02-simulation/urdf-xacro-models',
    'module-02-simulation/sensor-simulation',
    'module-02-simulation/ros2-gazebo-bridge',
    'module-02-simulation/unity-integration',
    'module-02-simulation/mini-project-walking-simulation',
  ],

  module3Sidebar: [
    'module-03-perception/01-introduction',
    'module-03-perception/02-computer-vision',
    'module-03-perception/03-speech-recognition',
    'module-03-perception/04-sensor-fusion',
    'module-03-perception/05-ros2-perception-integration',
    'module-03-perception/06-mini-project-object-identification',
  ],

  // Placeholder sidebars for other modules (content pending)
  module4Sidebar: [],
  capstoneSidebar: [],
};

export default sidebars;
