import React, { useState, useEffect } from 'react';
import ChatUI from '@site/src/components/ChatUI';
import { AuthProvider } from '@site/src/contexts/AuthProvider';
import { PersonalizationProvider } from '@site/src/components/PersonalizationProvider';
import { SimplePersonalizationProvider } from '@site/src/components/SimplePersonalizationProvider';

export default function Root({ children }) {
  const [selectedText, setSelectedText] = useState(null);
  const [showChat, setShowChat] = useState(false);

  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      // Only trigger if user selected substantial text (more than 10 characters)
      if (text.length > 10) {
        setSelectedText(text);
        setShowChat(true);
      }
    };

    // Add event listener for text selection
    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);
    };
  }, []);

  const handleCloseChat = () => {
    setShowChat(false);
    setSelectedText(null);
  };

  return (
    <AuthProvider>
      <PersonalizationProvider>
        <SimplePersonalizationProvider>
          {children}
          <ChatUI
            selectedText={selectedText}
            onClose={handleCloseChat}
            initialVisibility={showChat}
          />
        </SimplePersonalizationProvider>
      </PersonalizationProvider>
    </AuthProvider>
  );
}
