import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Book sidebar structure
  bookSidebar: [
    'introduction',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'chapters/module1-intro',
        'chapters/module1-architecture',
        'chapters/module1-communication',
        'chapters/module1-practice',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Technology',
      items: [
        'chapters/module2-intro',
        'chapters/module2-gazebo',
        'chapters/module2-unity',
        'chapters/module2-simtoreal',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      items: [
        'chapters/module3-intro',
        'chapters/module3-perception',
        'chapters/module3-sdk',
        'chapters/module3-deployment',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Models',
      items: [
        'chapters/module4-intro',
        'chapters/module4-multimodal',
        'chapters/module4-vla',
        'chapters/module4-future',
      ],
    },
    'summary',
    'glossary',
    'references',
  ],
};

export default sidebars;
