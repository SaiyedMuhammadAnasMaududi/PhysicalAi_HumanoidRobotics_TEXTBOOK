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

  // Placeholder sidebars for other modules (content pending)
  module2Sidebar: [],
  module3Sidebar: [],
  module4Sidebar: [],
  capstoneSidebar: [],
};

export default sidebars;
