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
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
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
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-01-ros2/introduction',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-02-simulation/introduction',
            },
            {
              label: 'Module 3: Perception',
              to: '/docs/module-03-perception/introduction',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'markup', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
