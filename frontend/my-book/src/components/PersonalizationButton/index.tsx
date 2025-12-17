import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { usePersonalization } from '@site/src/contexts/PersonalizationContext';
import { getBackendUrl } from '@site/src/utils/config';
import MarkdownRenderer from '@site/src/components/MarkdownRenderer/MarkdownRenderer';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './personalizationButton.css';

// Define TypeScript types
interface UserPreferences {
  learning_style: 'visual' | 'auditory' | 'reading/writing' | 'kinesthetic' | 'multimodal';
  experience_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  interests: string[];
  difficulty_preference: string;
  preferred_examples: string[];
  accessibility_needs: string[];
}

interface PersonalizationRequest {
  chapter_content: string;
  user_preferences: UserPreferences;
  chapter_id: string;
  book_id?: string;
}

interface PersonalizationResponse {
  original_chapter_id: string;
  personalized_content: string;
  processing_time_ms?: number;
  personalization_applied: boolean;
  warnings?: string[];
}

interface PersonalizationButtonProps {
  chapterId: string;
  chapterContent: string;
  onPersonalize?: (personalizedContent: string) => void;
}

const PersonalizationButtonContent: React.FC<PersonalizationButtonProps> = ({
  chapterId,
  chapterContent,
  onPersonalize
}) => {
  const {
    preferences,
    setPreferences,
    isPersonalized,
    getPersonalizedContent,
    setPersonalizedContent,
    isAuthenticated: userAuthenticated,
    loading: authLoading,
    userBackground
  } = usePersonalization();

  // Get backend URL from config utility
  const BACKEND_URL = getBackendUrl();

  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showPreferences, setShowPreferences] = useState(false);

  const hasBeenPersonalized = isPersonalized(chapterId);
  const personalizedContent = getPersonalizedContent(chapterId);

  const handlePersonalize = async () => {
    if (hasBeenPersonalized) return; // Prevent multiple clicks after application

    setIsLoading(true);
    setError(null);

    try {
      const request: PersonalizationRequest = {
        chapter_content: chapterContent,
        user_preferences: preferences,
        chapter_id: chapterId
      };

      const response = await fetch(`${BACKEND_URL}/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: PersonalizationResponse = await response.json();

      if (data.personalization_applied && data.personalized_content) {
        setPersonalizedContent(chapterId, data.personalized_content);
        if (onPersonalize) {
          onPersonalize(data.personalized_content);
        }
      } else {
        throw new Error('Personalization was not applied successfully');
      }
    } catch (err) {
      console.error('Personalization error:', err);
      setError(err instanceof Error ? err.message : 'An error occurred during personalization');
    } finally {
      setIsLoading(false);
    }
  };

  const handlePreferenceChange = (key: keyof UserPreferences, value: any) => {
    setPreferences({
      ...preferences,
      [key]: value
    });
  };

  const resetPreferences = () => {
    setPreferences({
      learning_style: 'multimodal',
      experience_level: 'intermediate',
      interests: [],
      difficulty_preference: 'moderate',
      preferred_examples: [],
      accessibility_needs: []
    });
  };

  // Show loading while checking auth
  if (authLoading) {
    return (
      <div className="personalization-container">
        <div className="personalization-header">
          <button className="personalize-btn" disabled>
            <span className="spinner"></span>
            Loading...
          </button>
        </div>
      </div>
    );
  }

  // Show sign-in prompt if not authenticated
  if (!userAuthenticated) {
    return (
      <div className="personalization-container">
        <div className="personalization-header">
          <button
            className="personalize-btn signin-required"
            onClick={() => window.location.href = '/signin?redirect=' + encodeURIComponent(window.location.pathname)}
            title="Sign in to personalize content"
          >
            🔒 Sign In to Personalize
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className="personalization-container">
      <div className="personalization-header">
        <button
          className={`personalize-btn ${hasBeenPersonalized ? 'applied' : ''} ${isLoading ? 'loading' : ''}`}
          onClick={handlePersonalize}
          disabled={isLoading || hasBeenPersonalized}
          title={hasBeenPersonalized ? 'Content already personalized' : 'Personalize this chapter based on your preferences'}
        >
          {isLoading ? (
            <>
              <span className="spinner"></span>
              Personalizing...
            </>
          ) : hasBeenPersonalized ? (
            <>
              ✅ Personalized
            </>
          ) : (
            <>
              🎯 Personalize Content
            </>
          )}
        </button>

        <button
          className="preferences-toggle"
          onClick={() => setShowPreferences(!showPreferences)}
          title="Adjust your learning preferences"
        >
          {showPreferences ? 'Hide Preferences' : 'Preferences'}
        </button>
      </div>

      {showPreferences && (
        <div className="preferences-panel">
          <div className="preferences-grid">
            <div className="preference-item">
              <label htmlFor="learning-style">Learning Style:</label>
              <select
                id="learning-style"
                value={preferences.learning_style}
                onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
              >
                <option value="visual">Visual</option>
                <option value="auditory">Auditory</option>
                <option value="reading/writing">Reading/Writing</option>
                <option value="kinesthetic">Kinesthetic</option>
                <option value="multimodal">Multimodal</option>
              </select>
            </div>

            <div className="preference-item">
              <label htmlFor="experience-level">Experience Level:</label>
              <select
                id="experience-level"
                value={preferences.experience_level}
                onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
                <option value="expert">Expert</option>
              </select>
            </div>

            <div className="preference-item">
              <label htmlFor="difficulty">Difficulty:</label>
              <select
                id="difficulty"
                value={preferences.difficulty_preference}
                onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
              >
                <option value="easy">Easy</option>
                <option value="moderate">Moderate</option>
                <option value="challenging">Challenging</option>
              </select>
            </div>
          </div>

          <div className="preferences-actions">
            <button onClick={resetPreferences} className="reset-btn">
              Reset to Defaults
            </button>
          </div>
        </div>
      )}

      {error && (
        <div className="error-message">
          Error: {error}
          <button onClick={() => setError(null)} className="close-error">
            ×
          </button>
        </div>
      )}

      {hasBeenPersonalized && personalizedContent && (
        <div className="personalized-content">
          <MarkdownRenderer content={personalizedContent} />
        </div>
      )}
    </div>
  );
};

const PersonalizationButton: React.FC<PersonalizationButtonProps> = (props) => {
  return (
    <BrowserOnly>
      {() => <PersonalizationButtonContent {...props} />}
    </BrowserOnly>
  );
};

export default PersonalizationButton;