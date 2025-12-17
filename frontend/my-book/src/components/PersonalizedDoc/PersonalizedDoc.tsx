import React, { useState, useEffect } from 'react';
import { useDoc } from '@docusaurus/theme-common/internal';
import PersonalizationButton from '../PersonalizationButton';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface PersonalizedDocProps {
  children: React.ReactNode;
}

const PersonalizedDoc: React.FC<PersonalizedDocProps> = ({ children }) => {
  const { metadata } = useDoc();
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<React.ReactNode | null>(null);

  // Store the original content
  useEffect(() => {
    setOriginalContent(children);
  }, [children]);

  // Get the chapter content as text for personalization
  const getChapterContent = (): string => {
    // In a real implementation, we would extract the actual text content
    // For now, we'll use a placeholder approach
    return metadata.title + "\n\n" + (metadata.description || "");
  };

  const handlePersonalize = (personalizedContent: string) => {
    setPersonalizedContent(personalizedContent);
  };

  // Function to render personalized content as JSX
  const renderPersonalizedContent = () => {
    if (!personalizedContent) return null;

    // This is a simplified approach - in a real implementation,
    // we would need to properly parse and render the Markdown content
    return (
      <div
        className="markdown"
        dangerouslySetInnerHTML={{ __html: personalizedContent }}
      />
    );
  };

  return (
    <div className="personalized-doc-container">
      <PersonalizationButton
        chapterId={metadata.unversionedId}
        chapterContent={getChapterContent()}
        onPersonalize={handlePersonalize}
      />

      {personalizedContent ? (
        <BrowserOnly>
          {() => renderPersonalizedContent()}
        </BrowserOnly>
      ) : (
        originalContent
      )}
    </div>
  );
};

export default PersonalizedDoc;