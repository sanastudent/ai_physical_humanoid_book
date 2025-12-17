import React, { useEffect, useState } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import rehypeRaw from 'rehype-raw';
import './markdownRenderer.css';

interface MarkdownRendererProps {
  content: string;
  className?: string;
}

const MarkdownRenderer: React.FC<MarkdownRendererProps> = ({ content, className = '' }) => {
  const [processedContent, setProcessedContent] = useState(content);

  // Process the content to ensure it's safe and properly formatted
  useEffect(() => {
    // Sanitize content if needed - in a real implementation,
    // you would use a proper sanitizer like DOMPurify
    setProcessedContent(content);
  }, [content]);

  // Custom components for ReactMarkdown to ensure proper rendering
  const components = {
    code({ node, inline, className, children, ...props }: any) {
      const match = /language-(\w+)/.exec(className || '');
      return !inline && match ? (
        <pre className={className} {...props}>
          <code className={className}>{children}</code>
        </pre>
      ) : (
        <code className={className} {...props}>
          {children}
        </code>
      );
    },
    // Ensure links open in the same tab to maintain navigation
    a({ node, href, children, ...props }: any) {
      return (
        <a href={href} target={href?.startsWith('http') ? '_blank' : undefined} rel={href?.startsWith('http') ? 'noopener noreferrer' : undefined} {...props}>
          {children}
        </a>
      );
    }
  };

  return (
    <div className={`markdown-renderer ${className}`}>
      <ReactMarkdown
        remarkPlugins={[remarkGfm]}
        rehypePlugins={[rehypeRaw]}
        components={components}
        children={processedContent}
      />
    </div>
  );
};

export default MarkdownRenderer;