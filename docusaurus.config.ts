import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Textbook for Building Autonomous Voice-Controlled Humanoid Robots',
  favicon: 'img/favicon.ico',

  url: 'https://saiyedmuhammadanasmaududi.github.io',
  baseUrl: '/PhysicalAi_HumanoidRobotics_TEXTBOOK/',

  organizationName: 'SaiyedMuhammadAnasMaududi',
  projectName: 'PhysicalAi_HumanoidRobotics_TEXTBOOK',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK/tree/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    announcementBar: {
      id: 'new_module',
      content:
        '🎉 <strong>Module 5: Capstone Project</strong> is now available! Build your complete voice-controlled humanoid robot. <a target="_blank" rel="noopener noreferrer" href="/docs/module-05-capstone/introduction">Get Started →</a>',
      backgroundColor: '#0066cc',
      textColor: '#ffffff',
      isCloseable: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          to: '/docs/intro',
          label: 'Home',
          position: 'left',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module1Sidebar',
          position: 'left',
          label: 'Module 1: ROS 2',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module2Sidebar',
          position: 'left',
          label: 'Module 2: Simulation',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module3Sidebar',
          position: 'left',
          label: 'Module 3: Perception',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module4Sidebar',
          position: 'left',
          label: 'Module 4: Control',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module5Sidebar',
          position: 'left',
          label: 'Module 5: Capstone',
        },
        {
          href: 'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning Modules',
          items: [
            {
              label: 'Module 1: ROS 2 Fundamentals',
              to: '/docs/module-01-ros2/introduction',
            },
            {
              label: 'Module 2: Simulation & Digital Twins',
              to: '/docs/module-02-simulation/introduction',
            },
            {
              label: 'Module 3: Perception Systems',
              to: '/docs/module-03-perception/introduction',
            },
            {
              label: 'Module 4: Planning & Control',
              to: '/docs/module-04-control/introduction',
            },
            {
              label: 'Module 5: Capstone Project',
              to: '/docs/module-05-capstone/introduction',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Gazebo Simulator',
              href: 'https://gazebosim.org/',
            },
            {
              label: 'OpenCV Tutorials',
              href: 'https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html',
            },
            {
              label: 'OpenAI Whisper',
              href: 'https://github.com/openai/whisper',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK',
            },
            {
              label: 'Report Issues',
              href: 'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK/issues',
            },
            {
              label: 'Contribute',
              href: 'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK/blob/main/CONTRIBUTING.md',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'About This Textbook',
              to: '/docs/intro',
            },
            {
              label: 'License (MIT)',
              href: 'https://github.com/SaiyedMuhammadAnasMaududi/PhysicalAi_HumanoidRobotics_TEXTBOOK/blob/main/LICENSE',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with ❤️ using Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'markup', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
