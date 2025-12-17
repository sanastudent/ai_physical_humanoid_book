import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type {WrapperProps} from '@docusaurus/types';
import BrowserOnly from '@docusaurus/BrowserOnly';
import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslateButton from '@site/src/components/TranslateButton';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  return (
    <>
      <BrowserOnly>
        {() => {
          // Get the current doc content
          const chapterId = props.content.metadata.id;
          const chapterTitle = props.content.metadata.title;

          // Get the content element after render
          React.useEffect(() => {
            const articleElement = document.querySelector('article');
            if (!articleElement) return;

            // Extract text content from the article
            const chapterContent = articleElement.innerText || '';

            // Check if buttons already exist
            const existingButtonsContainer = document.getElementById('chapter-actions-container');
            if (existingButtonsContainer) return;

            // Create container for buttons
            const buttonsContainer = document.createElement('div');
            buttonsContainer.id = 'chapter-actions-container';
            buttonsContainer.style.cssText = 'display: flex; gap: 10px; margin: 20px 0; padding: 15px; background-color: #f5f5f5; border-radius: 8px; border: 1px solid #ddd;';

            // Insert buttons container at the top of the article
            if (articleElement.firstChild) {
              articleElement.insertBefore(buttonsContainer, articleElement.firstChild);
            } else {
              articleElement.appendChild(buttonsContainer);
            }

            // Render PersonalizationButton WITH PersonalizationProvider
            const personalizeContainer = document.createElement('div');
            buttonsContainer.appendChild(personalizeContainer);

            const { createRoot } = require('react-dom/client');
            const personalizeRoot = createRoot(personalizeContainer);
            personalizeRoot.render(
              <PersonalizationButton
                chapterId={chapterId}
                chapterContent={chapterContent}
              />
            );

            // Render TranslateButton (no provider needed)
            const translateContainer = document.createElement('div');
            buttonsContainer.appendChild(translateContainer);

            const translateRoot = createRoot(translateContainer);
            translateRoot.render(
              <TranslateButton
                chapterId={chapterId}
                chapterContent={chapterContent}
              />
            );

            // Cleanup function
            return () => {
              if (existingButtonsContainer) {
                existingButtonsContainer.remove();
              }
            };
          }, [chapterId]);

          return null;
        }}
      </BrowserOnly>
      <DocItem {...props} />
    </>
  );
}
