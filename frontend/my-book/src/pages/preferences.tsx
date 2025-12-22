import React from 'react';
import Layout from '@theme/Layout';
import { usePersonalization } from '@site/src/components/PersonalizationProvider';
import './preferences.css';

const PreferencesPageContent: React.FC = () => {
  const {
    preferences,
    setPreferences,
    addInterest,
    removeInterest
  } = usePersonalization();

  const [originalPreferences, setOriginalPreferences] = React.useState(preferences);
  const [hasChanges, setHasChanges] = React.useState(false);
  const [saveMessage, setSaveMessage] = React.useState<string | null>(null);

  React.useEffect(() => {
    const changed = JSON.stringify(preferences) !== JSON.stringify(originalPreferences);
    setHasChanges(changed);
  }, [preferences, originalPreferences]);

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

  const handleSave = () => {
    setOriginalPreferences(preferences);
    setHasChanges(false);
    setSaveMessage('Preferences saved successfully!');
    setTimeout(() => setSaveMessage(null), 3000);
  };

  const handleCancel = () => {
    setPreferences(originalPreferences);
    setHasChanges(false);
    setSaveMessage(null);
  };

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>Learning Preferences</h1>
          <p>Customize your learning experience by setting your preferences below. Changes are saved to your browser and will personalize your content.</p>

          {saveMessage && (
            <div className="save-message success">
              {saveMessage}
            </div>
          )}

          <div className="preferences-form">
            <div className="preference-section">
              <h2>Learning Style</h2>
              <p className="section-description">Choose how you learn best. This helps tailor content delivery to match your preferred learning method.</p>
              <div className="radio-group">
                <label className={preferences.learning_style === 'visual' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Visual</span>
                    <span className="option-detail">Diagrams, charts, and visual aids</span>
                  </span>
                  <input
                    type="radio"
                    name="learning_style"
                    value="visual"
                    checked={preferences.learning_style === 'visual'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                </label>
                <label className={preferences.learning_style === 'auditory' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Auditory</span>
                    <span className="option-detail">Discussions, lectures, and audio</span>
                  </span>
                  <input
                    type="radio"
                    name="learning_style"
                    value="auditory"
                    checked={preferences.learning_style === 'auditory'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                </label>
                <label className={preferences.learning_style === 'reading/writing' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Reading/Writing</span>
                    <span className="option-detail">Text-based content</span>
                  </span>
                  <input
                    type="radio"
                    name="learning_style"
                    value="reading/writing"
                    checked={preferences.learning_style === 'reading/writing'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                </label>
                <label className={preferences.learning_style === 'kinesthetic' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Kinesthetic</span>
                    <span className="option-detail">Hands-on and practical</span>
                  </span>
                  <input
                    type="radio"
                    name="learning_style"
                    value="kinesthetic"
                    checked={preferences.learning_style === 'kinesthetic'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                </label>
                <label className={preferences.learning_style === 'multimodal' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Multimodal</span>
                    <span className="option-detail">Mixed approach</span>
                  </span>
                  <input
                    type="radio"
                    name="learning_style"
                    value="multimodal"
                    checked={preferences.learning_style === 'multimodal'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>Experience Level</h2>
              <p className="section-description">Select your current knowledge level. This adjusts the complexity and depth of explanations.</p>
              <div className="radio-group">
                <label className={preferences.experience_level === 'beginner' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Beginner</span>
                  </span>
                  <input
                    type="radio"
                    name="experience_level"
                    value="beginner"
                    checked={preferences.experience_level === 'beginner'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                </label>
                <label className={preferences.experience_level === 'intermediate' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Intermediate</span>
                  </span>
                  <input
                    type="radio"
                    name="experience_level"
                    value="intermediate"
                    checked={preferences.experience_level === 'intermediate'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                </label>
                <label className={preferences.experience_level === 'advanced' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Advanced</span>
                  </span>
                  <input
                    type="radio"
                    name="experience_level"
                    value="advanced"
                    checked={preferences.experience_level === 'advanced'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                </label>
                <label className={preferences.experience_level === 'expert' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Expert</span>
                  </span>
                  <input
                    type="radio"
                    name="experience_level"
                    value="expert"
                    checked={preferences.experience_level === 'expert'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>Difficulty Preference</h2>
              <p className="section-description">Choose your preferred challenge level for exercises and examples.</p>
              <div className="radio-group">
                <label className={preferences.difficulty_preference === 'easy' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Easy</span>
                  </span>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="easy"
                    checked={preferences.difficulty_preference === 'easy'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                </label>
                <label className={preferences.difficulty_preference === 'moderate' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Moderate</span>
                  </span>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="moderate"
                    checked={preferences.difficulty_preference === 'moderate'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                </label>
                <label className={preferences.difficulty_preference === 'challenging' ? 'selected' : ''}>
                  <span className="custom-radio"></span>
                  <span className="option-content">
                    <span className="option-label">Challenging</span>
                  </span>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="challenging"
                    checked={preferences.difficulty_preference === 'challenging'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
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
              <div className="actions-left">
                <button
                  onClick={resetPreferences}
                  className="button button--secondary button--outline"
                >
                  Reset to Defaults
                </button>
              </div>
              <div className="actions-right">
                {hasChanges && (
                  <>
                    <button
                      onClick={handleCancel}
                      className="button button--secondary"
                    >
                      Cancel
                    </button>
                    <button
                      onClick={handleSave}
                      className="button button--primary"
                    >
                      Save Changes
                    </button>
                  </>
                )}
                {!hasChanges && (
                  <span className="no-changes-message">All changes saved</span>
                )}
              </div>
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
      <PreferencesPageContent />
    </Layout>
  );
};

export default PreferencesPage;