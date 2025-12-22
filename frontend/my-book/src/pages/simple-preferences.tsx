import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useSimplePersonalization } from '@site/src/components/SimplePersonalizationProvider';

const SimplePreferencesPageContent: React.FC = () => {
  const { preferences, setPreferences, addInterest, removeInterest, setIsPersonalized } = useSimplePersonalization();
  const [newInterest, setNewInterest] = useState('');

  const handleAddInterest = (e: React.FormEvent) => {
    e.preventDefault();
    if (newInterest.trim() && !preferences.interests.includes(newInterest.trim())) {
      addInterest(newInterest.trim());
      setNewInterest('');
    }
  };

  const handleRemoveInterest = (interest: string) => {
    removeInterest(interest);
  };

  const handleExperienceChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setPreferences({
      ...preferences,
      experience_level: e.target.value
    });
  };

  const handlePreferredExamplesChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const example = e.target.value;
    if (!preferences.preferred_examples.includes(example)) {
      setPreferences({
        ...preferences,
        preferred_examples: [...preferences.preferred_examples, example]
      });
    }
  };

  const markAsPersonalized = () => {
    setIsPersonalized(true);
  };

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>Simple Preferences</h1>
          <p>Set your interests to personalize your learning experience.</p>

          <div className="card margin-vert--lg">
            <div className="card__body">
              <h2>Interests</h2>
              <form onSubmit={handleAddInterest} className="margin-vert--md">
                <div className="row">
                  <div className="col col--8">
                    <input
                      type="text"
                      value={newInterest}
                      onChange={(e) => setNewInterest(e.target.value)}
                      placeholder="Add an interest (e.g., robotics, AI, web development)"
                      className="form-control"
                    />
                  </div>
                  <div className="col col--4">
                    <button type="submit" className="button button--primary">
                      Add Interest
                    </button>
                  </div>
                </div>
              </form>

              <div className="margin-vert--md">
                {preferences.interests.map((interest, index) => (
                  <span key={index} className="badge badge--secondary margin-right--sm margin-bottom--sm">
                    {interest}
                    <button
                      onClick={() => handleRemoveInterest(interest)}
                      className="margin-left--sm"
                      style={{ background: 'none', border: 'none', color: 'white', cursor: 'pointer' }}
                    >
                      ×
                    </button>
                  </span>
                ))}
              </div>

              <div className="margin-vert--md">
                <label htmlFor="experience">Experience Level: </label>
                <select
                  id="experience"
                  value={preferences.experience_level}
                  onChange={handleExperienceChange}
                  className="margin-left--sm"
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                  <option value="expert">Expert</option>
                </select>
              </div>

              <div className="margin-vert--md">
                <label htmlFor="examples">Preferred Examples: </label>
                <select
                  id="examples"
                  onChange={handlePreferredExamplesChange}
                  className="margin-left--sm"
                  defaultValue=""
                >
                  <option value="" disabled>Select example type</option>
                  <option value="robotics">Robotics</option>
                  <option value="ai">Artificial Intelligence</option>
                  <option value="web-dev">Web Development</option>
                  <option value="data-science">Data Science</option>
                  <option value="mobile">Mobile Development</option>
                </select>
              </div>

              <div className="margin-vert--md">
                <button
                  className="button button--success"
                  onClick={markAsPersonalized}
                >
                  Mark as Personalized
                </button>
              </div>
            </div>
          </div>

          <div className="card margin-vert--lg">
            <div className="card__body">
              <h3>Current Preferences</h3>
              <p><strong>Interests:</strong> {preferences.interests.length > 0 ? preferences.interests.join(', ') : 'None set'}</p>
              <p><strong>Experience Level:</strong> {preferences.experience_level}</p>
              <p><strong>Preferred Examples:</strong> {preferences.preferred_examples.length > 0 ? preferences.preferred_examples.join(', ') : 'None set'}</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const SimplePreferencesPage: React.FC = () => {
  return (
    <Layout title="Simple Preferences" description="Set your simple preferences for personalization">
      <SimplePreferencesPageContent />
    </Layout>
  );
};

export default SimplePreferencesPage;