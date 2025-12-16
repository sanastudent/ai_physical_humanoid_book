import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Homepage(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="فزیکل اے آئی اور ہیومنائیڈ"
      description="فزیکل اے آئی اور ہیومنائیڈ روبوٹکس پر ایک جامع کتاب">

      <main className={styles.hero}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            فزیکل اے آئی اور ہیومنائیڈ
          </h1>

          <Link
            to={useBaseUrl('/docs/chapters/module1-intro')}
            className={styles.ctaButton}
          >
            شروع کریں
          </Link>
        </div>
      </main>
    </Layout>
  );
}
