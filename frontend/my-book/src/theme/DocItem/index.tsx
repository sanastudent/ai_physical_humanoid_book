import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type {WrapperProps} from '@docusaurus/types';
import BrowserOnly from '@docusaurus/BrowserOnly';
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

            // Check if button already exists
            const existingButtonContainer = document.getElementById('chapter-actions-container');
            if (existingButtonContainer) return;

            // Create container for button
            const buttonContainer = document.createElement('div');
            buttonContainer.id = 'chapter-actions-container';
            buttonContainer.style.cssText = 'display: flex; gap: 10px; margin: 20px 0; padding: 15px; background-color: #f5f5f5; border-radius: 8px; border: 1px solid #ddd;';

            // Insert button container at the top of the article
            if (articleElement.firstChild) {
              articleElement.insertBefore(buttonContainer, articleElement.firstChild);
            } else {
              articleElement.appendChild(buttonContainer);
            }

            // Render TranslateButton
            const translateContainer = document.createElement('div');
            buttonContainer.appendChild(translateContainer);

            const { createRoot } = require('react-dom/client');
            const translateRoot = createRoot(translateContainer);
            translateRoot.render(
              <TranslateButton
                chapterId={chapterId}
                chapterContent={chapterContent}
              />
            );

            // Cleanup function
            return () => {
              if (existingButtonContainer) {
                existingButtonContainer.remove();
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
