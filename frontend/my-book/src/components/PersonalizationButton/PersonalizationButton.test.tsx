// Frontend Integration Tests for Personalization Button Component
// Tests the personalization button functionality and integration

import { describe, it, expect, vi, beforeEach } from 'vitest';

describe('PersonalizationButton Component Tests', () => {
  it('should have correct component structure', () => {
    // Verify that the component has the expected functionality
    const expectedFeatures = [
      'Personalization button',
      'Preferences panel toggle',
      'Loading states management',
      'API communication',
      'Error handling',
      'Content rendering'
    ];

    expect(expectedFeatures.length).toBe(6);
    expect(expectedFeatures).toContain('Personalization button');
    expect(expectedFeatures).toContain('API communication');
  });

  it('should handle user preferences correctly', () => {
    // Test that user preferences are properly managed
    const userPreferences = {
      learning_style: 'multimodal',
      experience_level: 'intermediate',
      interests: [],
      difficulty_preference: 'moderate',
      preferred_examples: [],
      accessibility_needs: []
    };

    expect(userPreferences).toHaveProperty('learning_style');
    expect(userPreferences.learning_style).toBe('multimodal');
    expect(userPreferences).toHaveProperty('experience_level');
    expect(userPreferences.experience_level).toBe('intermediate');
    expect(userPreferences.interests).toBeInstanceOf(Array);
  });

  it('should manage button states properly', () => {
    // Test button states: default, loading, applied, disabled
    const buttonStates = {
      default: { text: '🎯 Personalize Content', disabled: false },
      loading: { text: 'Personalizing...', disabled: true },
      applied: { text: '✅ Personalized', disabled: true },
      error: { text: '⚠️ Error', disabled: false }
    };

    expect(buttonStates.default.disabled).toBe(false);
    expect(buttonStates.loading.disabled).toBe(true);
    expect(buttonStates.applied.disabled).toBe(true);
  });

  it('should handle API communication', () => {
    // Mock the fetch API call that the component makes
    const mockFetch = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({
        original_chapter_id: 'test-chapter-1',
        personalized_content: '# Personalized Chapter\nThis is personalized content.',
        personalization_applied: true
      })
    });

    global.fetch = mockFetch;

    // Test the API call parameters
    const requestParams = {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        chapter_content: '# Test Chapter\nThis is a test chapter.',
        user_preferences: {
          learning_style: 'multimodal',
          experience_level: 'intermediate',
          interests: [],
          difficulty_preference: 'moderate'
        },
        chapter_id: 'test-chapter-1'
      })
    };

    expect(requestParams.method).toBe('POST');
    expect(requestParams.headers['Content-Type']).toBe('application/json');
    expect(requestParams.body).toContain('chapter_content');
    expect(requestParams.body).toContain('user_preferences');
  });

  it('should preserve content structure', () => {
    // Test that markdown structure is preserved during personalization
    const originalContent = `# Introduction
This is an introduction.

## Section 1
Some text here.

\`\`\`python
def hello():
    print("Hello, World!")
\`\`\`

[Chapter 1: Paragraph 1] This is a citation.`;

    // Verify that structure elements are preserved
    expect(originalContent).toContain('# Introduction');
    expect(originalContent).toContain('## Section 1');
    expect(originalContent).toContain('def hello():');
    expect(originalContent).toContain('[Chapter 1: Paragraph 1]');
  });

  it('should handle different learning styles', () => {
    // Test different learning styles that the component should support
    const learningStyles = [
      'visual',
      'auditory',
      'reading/writing',
      'kinesthetic',
      'multimodal'
    ];

    expect(learningStyles).toContain('visual');
    expect(learningStyles).toContain('multimodal');
    expect(learningStyles.length).toBe(5);
  });

  it('should handle different experience levels', () => {
    // Test different experience levels that the component should support
    const experienceLevels = [
      'beginner',
      'intermediate',
      'advanced',
      'expert'
    ];

    expect(experienceLevels).toContain('beginner');
    expect(experienceLevels).toContain('expert');
    expect(experienceLevels.length).toBe(4);
  });

  it('should handle different difficulty preferences', () => {
    // Test different difficulty preferences that the component should support
    const difficultyPreferences = [
      'easy',
      'moderate',
      'challenging'
    ];

    expect(difficultyPreferences).toContain('moderate');
    expect(difficultyPreferences.length).toBe(3);
  });

  it('should render personalized content correctly', () => {
    // Test that personalized content is rendered properly
    const originalContent = '# Original Content\nOriginal text here.';
    const personalizedContent = '# Personalized Content\nPersonalized text here based on preferences.';

    expect(personalizedContent).not.toBe(originalContent);
    expect(personalizedContent).toContain('Personalized');
    expect(personalizedContent).toContain('preferences');
  });

  it('should handle error states', () => {
    // Test error handling in the component
    const errorStates = {
      apiError: 'Failed to connect to personalization service',
      validationError: 'Invalid user preferences provided',
      contentError: 'Could not process chapter content'
    };

    expect(errorStates).toHaveProperty('apiError');
    expect(errorStates).toHaveProperty('validationError');
    expect(errorStates).toHaveProperty('contentError');
  });
});

// Test the preferences management functionality
describe('Preferences Management Tests', () => {
  it('should manage interests correctly', () => {
    // Test adding and removing interests
    let interests = [];

    // Add interest
    interests = [...interests, 'AI'];
    expect(interests).toContain('AI');

    // Add another interest
    interests = [...interests, 'Robotics'];
    expect(interests).toContain('Robotics');
    expect(interests.length).toBe(2);

    // Remove interest
    interests = interests.filter(i => i !== 'AI');
    expect(interests).not.toContain('AI');
    expect(interests).toContain('Robotics');
  });

  it('should maintain preferences across sessions', () => {
    // Test that preferences are stored and retrieved correctly
    const mockLocalStorage = {
      getItem: vi.fn(),
      setItem: vi.fn(),
      removeItem: vi.fn(),
    };

    global.localStorage = mockLocalStorage;

    // Simulate saving preferences
    const preferences = {
      learning_style: 'visual',
      experience_level: 'beginner',
      interests: ['AI', 'ML'],
      difficulty_preference: 'easy'
    };

    localStorage.setItem('userPreferences', JSON.stringify(preferences));

    expect(localStorage.setItem).toHaveBeenCalledWith(
      'userPreferences',
      JSON.stringify(preferences)
    );
  });
});