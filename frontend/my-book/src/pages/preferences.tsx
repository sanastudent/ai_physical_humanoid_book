import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Layout from '@theme/Layout';
import './preferences.css';

const PreferencesPageContent: React.FC = () => {
  const { usePersonalization } = require('@site/src/contexts/PersonalizationContext');
  const {
    preferences,
    setPreferences,
    addInterest,
    removeInterest
  } = usePersonalization();

  const handlePreferenceChange = (key: keyof typeof preferences, value: any) => {
    setPreferences({
      ...preferences,
      [key]: value
    });
  };

  const handleAddInterest = (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const formData = new FormData(e.currentTarget);
    const newInterest = formData.get('interest') as string;
    if (newInterest && !preferences.interests.includes(newInterest)) {
      addInterest(newInterest);
    }
    e.currentTarget.reset();
  };

  const handleRemoveInterest = (interest: string) => {
    // Create new array without the specified interest
    const newInterests = preferences.interests.filter(i => i !== interest);
    setPreferences({
      ...preferences,
      interests: newInterests
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

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>Learning Preferences</h1>
          <p>Customize your learning experience by setting your preferences below.</p>

          <div className="preferences-form">
            <div className="preference-section">
              <h2>Learning Style</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="visual"
                    checked={preferences.learning_style === 'visual'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  Visual (diagrams, charts, visual aids)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="auditory"
                    checked={preferences.learning_style === 'auditory'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  Auditory (discussions, lectures, audio)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="reading/writing"
                    checked={preferences.learning_style === 'reading/writing'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  Reading/Writing (text-based content)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="kinesthetic"
                    checked={preferences.learning_style === 'kinesthetic'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  Kinesthetic (hands-on, practical)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="multimodal"
                    checked={preferences.learning_style === 'multimodal'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  Multimodal (mixed approach)
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>Experience Level</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="beginner"
                    checked={preferences.experience_level === 'beginner'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  Beginner
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="intermediate"
                    checked={preferences.experience_level === 'intermediate'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  Intermediate
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="advanced"
                    checked={preferences.experience_level === 'advanced'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  Advanced
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="expert"
                    checked={preferences.experience_level === 'expert'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  Expert
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>Difficulty Preference</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="easy"
                    checked={preferences.difficulty_preference === 'easy'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  Easy
                </label>
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="moderate"
                    checked={preferences.difficulty_preference === 'moderate'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  Moderate
                </label>
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="challenging"
                    checked={preferences.difficulty_preference === 'challenging'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  Challenging
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>Interests</h2>
              <p>Add topics or areas you're interested in to customize examples and content:</p>
              <form onSubmit={handleAddInterest} className="interests-form">
                <input
                  type="text"
                  name="interest"
                  placeholder="Add an interest (e.g., robotics, AI, Python)"
                  className="input-field"
                />
                <button type="submit" className="button button--primary">Add</button>
              </form>

              <div className="interests-list">
                {preferences.interests.map((interest, index) => (
                  <span key={index} className="interest-tag">
                    {interest}
                    <button
                      onClick={() => handleRemoveInterest(interest)}
                      className="remove-interest"
                      aria-label={`Remove ${interest}`}
                    >
                      ×
                    </button>
                  </span>
                ))}
              </div>
            </div>

            <div className="preference-actions">
              <button
                onClick={resetPreferences}
                className="button button--secondary"
              >
                Reset to Defaults
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const PreferencesPage: React.FC = () => {
  return (
    <Layout title="Learning Preferences" description="Manage your personal learning preferences">
      <BrowserOnly>
        {() => <PreferencesPageContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default PreferencesPage;