import type {ReactNode} from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { translate } from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {translate({message: 'Welcome to My Book!'})}
        </Heading>
        <p className="hero__subtitle">{translate({message: 'Start exploring the content'})}</p>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={translate({message: `Welcome to ${siteConfig.title}`})}
      description={translate({message: 'A comprehensive guide to building intelligent robotic systems'})}>
      <HomepageHeader />
      <main className={styles.mainContent}>
        {/* Clean and minimal - no feature sections */}
      </main>
    </Layout>
  );
}
