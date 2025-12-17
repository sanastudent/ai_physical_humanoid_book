import React, { useState, useEffect, useRef } from 'react';
import './chatui.css';

// Define TypeScript types
type Message = {
  id?: number;
  role: 'user' | 'assistant' | 'system';
  content: string;
  type?: 'context';
  error?: boolean;
  citations?: string[];
  sources?: string[];
};

type ChatUIProps = {
  selectedText?: string | null;
  onClose?: () => void;
  initialVisibility?: boolean;
};

// Since we can't import CSS from static directory in a component,
// we'll dynamically load it or define styles in the component
// For now, we'll assume the CSS is available globally or copy the essential styles
const ChatUI: React.FC<ChatUIProps> = ({ selectedText = null, onClose, initialVisibility = false }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isVisible, setIsVisible] = useState(initialVisibility);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000'; // Default backend URL

  useEffect(() => {
    if (selectedText) {
      setIsVisible(true);
      setMessages([
        {
          role: 'system',
          content: `Context: "${selectedText}"`,
          type: 'context'
        }
      ]);
    }
  }, [selectedText]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const endpoint = selectedText ? '/select' : '/query';
      const payload = {
        query: input,
        mode: selectedText ? 'selected' : 'global',
        context: selectedText || undefined
      };

      const response = await fetch(`${backendUrl}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Create an assistant message with empty content to simulate streaming
      const assistantMessageId = Date.now();
      const initialAssistantMessage: Message = {
        id: assistantMessageId,
        role: 'assistant',
        content: '',
        citations: [],
        sources: []
      };

      setMessages(prev => [...prev, initialAssistantMessage]);

      const data = await response.json();

      // Simulate streaming by typing out the response character by character
      const fullAnswer = data.answer;
      const fullCitations = data.citations || [];
      const fullSources = data.sources || [];

      // Update the message character by character to simulate streaming
      for (let i = 0; i <= fullAnswer.length; i++) {
        await new Promise(resolve => setTimeout(resolve, 20)); // 20ms per character
        setMessages(prev =>
          prev.map(msg =>
            msg.id === assistantMessageId
              ? {
                  ...msg,
                  content: fullAnswer.substring(0, i)
                }
              : msg
          )
        );
      }

      // After the content is "typed", add the citations and sources
      setMessages(prev =>
        prev.map(msg =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                citations: fullCitations,
                sources: fullSources
              }
            : msg
        )
      );
    } catch (error) {
      console.error('Error querying RAG:', error);
      // Remove the initial empty assistant message and add error message
      setMessages(prev => {
        const updatedMessages = prev.filter(msg => !msg.id);
        updatedMessages.push({
          role: 'assistant',
          content: 'Sorry, I encountered an error processing your question. Please make sure the backend server is running.',
          error: true
        });
        return updatedMessages;
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleVisibility = () => {
    setIsVisible(!isVisible);
  };

  const handleClose = () => {
    setIsVisible(false);
    if (onClose) onClose();
  };

  const renderMessage = (message: Message, index: number) => {
    if (message.type === 'context') {
      return (
        <div key={index} className="message context-message">
          <div className="message-content">
            <strong>Selected Text Context:</strong>
            <p>{message.content}</p>
          </div>
        </div>
      );
    }

    return (
      <div key={index} className={`message ${message.role}-message ${message.error ? 'error' : ''}`}>
        <div className="message-content">
          <div className="message-text">{message.content}</div>
          {message.citations && message.citations.length > 0 && (
            <div className="citations">
              <strong>Citations:</strong>
              <ul>
                {message.citations.map((citation, i) => (
                  <li key={i}>{citation}</li>
                ))}
              </ul>
            </div>
          )}
        </div>
      </div>
    );
  };

  return (
    <>
      {!isVisible && (
        <button className="chat-toggle-button" onClick={toggleVisibility}>
          💬 Ask about this book
        </button>
      )}

      {isVisible && (
        <div className="chat-container">
          <div className="chat-header">
            <h3>📚 Book Assistant</h3>
            <div className="chat-controls">
              {selectedText && <span className="badge">Selected Text Mode</span>}
              <button className="close-button" onClick={handleClose}>×</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="welcome-message">
                <h4>Welcome! 👋</h4>
                <p>Ask me anything about the book. I'll provide answers with citations.</p>
                {selectedText ? (
                  <p className="mode-info">You're in <strong>Selected Text Mode</strong> - I'll answer based on your highlighted text.</p>
                ) : (
                  <p className="mode-info">You're in <strong>Global Mode</strong> - I can answer questions about the entire book.</p>
                )}
              </div>
            )}
            {messages.map((message, index) => renderMessage(message, index))}
            {isLoading && (
              <div className="message assistant-message loading-message">
                <div className="loading-indicator">
                  <div className="dot"></div>
                  <div className="dot"></div>
                  <div className="dot"></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-container">
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the book..."
              rows={2}
              disabled={isLoading}
            />
            <button
              onClick={handleSendMessage}
              disabled={isLoading || !input.trim()}
              className="send-button"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatUI;