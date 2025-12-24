import '@testing-library/jest-dom/vitest';

// Mock the Docusaurus context and modules
vi.mock('@site/src/contexts/PersonalizationContext', async () => {
  const React = await import('react');
  const actual = await vi.importActual('@site/src/contexts/PersonalizationContext');
  return {
    ...actual,
    usePersonalization: () => ({
      preferences: {
        learning_style: 'multimodal',
        experience_level: 'intermediate',
        interests: [],
        difficulty_preference: 'moderate',
        preferred_examples: [],
        accessibility_needs: []
      },
      setPreferences: vi.fn(),
      isPersonalized: vi.fn().mockReturnValue(false),
      getPersonalizedContent: vi.fn().mockReturnValue(null),
      setPersonalizedContent: vi.fn(),
      addInterest: vi.fn(),
      removeInterest: vi.fn(),
    }),
    PersonalizationProvider: ({ children }: { children: React.ReactNode }) => React.createElement(React.Fragment, null, children),
  };
});

// Mock the MarkdownRenderer component
vi.mock('@site/src/components/MarkdownRenderer/MarkdownRenderer', async () => {
  const React = await import('react');
  return {
    default: ({ content }: { content: string }) => React.createElement('div', { 'data-testid': 'markdown-renderer' }, content)
  };
});

// Mock the environment variables
Object.defineProperty(window, 'process', {
  value: {
    env: {
      REACT_APP_BACKEND_URL: 'http://localhost:8000'
    }
  },
  writable: true
});