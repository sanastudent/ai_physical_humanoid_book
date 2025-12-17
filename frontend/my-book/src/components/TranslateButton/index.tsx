import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getBackendUrl } from '@site/src/utils/config';
import MarkdownRenderer from '@site/src/components/MarkdownRenderer/MarkdownRenderer';
import './translateButton.css';

interface TranslateButtonProps {
  chapterId: string;
  chapterContent: string;
  onTranslate?: (translatedContent: string) => void;
}

interface TranslateResponse {
  original_chapter_id: string;
  translated_content: string;
  target_language: string;
  processing_time_ms?: number;
}

const TranslateButtonContent: React.FC<TranslateButtonProps> = ({
  chapterId,
  chapterContent,
  onTranslate
}) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (isTranslated) {
      // Toggle back to original
      setIsTranslated(false);
      setTranslatedContent(null);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Get backend URL from config utility
      const backendUrl = getBackendUrl();

      const response = await fetch(`${backendUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: chapterContent,
          target_language: 'urdu',
          chapter_id: chapterId
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: TranslateResponse = await response.json();

      if (data.translated_content) {
        setTranslatedContent(data.translated_content);
        setIsTranslated(true);
        if (onTranslate) {
          onTranslate(data.translated_content);
        }
      } else {
        throw new Error('Translation was not successful');
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError(err instanceof Error ? err.message : 'An error occurred during translation');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="translate-container">
      <button
        className={`translate-btn ${isTranslated ? 'active' : ''} ${isLoading ? 'loading' : ''}`}
        onClick={handleTranslate}
        disabled={isLoading}
        title={isTranslated ? 'Show original English content' : 'Translate to Urdu'}
      >
        {isLoading ? (
          <>
            <span className="spinner"></span>
            Translating...
          </>
        ) : isTranslated ? (
          <>
            🇬🇧 English
          </>
        ) : (
          <>
            🇵🇰 اردو
          </>
        )}
      </button>

      {error && (
        <div className="error-message">
          Error: {error}
          <button onClick={() => setError(null)} className="close-error">
            ×
          </button>
        </div>
      )}

      {isTranslated && translatedContent && (
        <div className="translated-content">
          <div className="translation-badge">
            <span>🇵🇰 Translated to Urdu</span>
          </div>
          <MarkdownRenderer content={translatedContent} />
        </div>
      )}
    </div>
  );
};

const TranslateButton: React.FC<TranslateButtonProps> = (props) => {
  return (
    <BrowserOnly>
      {() => <TranslateButtonContent {...props} />}
    </BrowserOnly>
  );
};

export default TranslateButton;
