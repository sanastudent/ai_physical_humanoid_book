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
          <h1>سیکھنے کی ترجیحات</h1>
          <p>نیچے اپنی ترجیحات سیٹ کرکے اپنے سیکھنے کے تجربے کو اپنی مرضی کے مطابق بنائیں۔</p>

          <div className="preferences-form">
            <div className="preference-section">
              <h2>سیکھنے کا انداز</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="visual"
                    checked={preferences.learning_style === 'visual'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  بصری (ڈایاگرام، چارٹ، بصری معاون)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="auditory"
                    checked={preferences.learning_style === 'auditory'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  سماعتی (مباحثے، لیکچرز، آڈیو)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="reading/writing"
                    checked={preferences.learning_style === 'reading/writing'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  پڑھنا/لکھنا (متن پر مبنی مواد)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="kinesthetic"
                    checked={preferences.learning_style === 'kinesthetic'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  حرکی (عملی، ہاتھ سے کام)
                </label>
                <label>
                  <input
                    type="radio"
                    name="learning_style"
                    value="multimodal"
                    checked={preferences.learning_style === 'multimodal'}
                    onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                  />
                  کثیر الاقسام (مخلوط انداز)
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>تجربے کی سطح</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="beginner"
                    checked={preferences.experience_level === 'beginner'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  ابتدائی
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="intermediate"
                    checked={preferences.experience_level === 'intermediate'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  درمیانی
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="advanced"
                    checked={preferences.experience_level === 'advanced'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  اعلیٰ
                </label>
                <label>
                  <input
                    type="radio"
                    name="experience_level"
                    value="expert"
                    checked={preferences.experience_level === 'expert'}
                    onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  />
                  ماہر
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>مشکل کی ترجیح</h2>
              <div className="radio-group">
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="easy"
                    checked={preferences.difficulty_preference === 'easy'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  آسان
                </label>
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="moderate"
                    checked={preferences.difficulty_preference === 'moderate'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  اعتدال پسند
                </label>
                <label>
                  <input
                    type="radio"
                    name="difficulty_preference"
                    value="challenging"
                    checked={preferences.difficulty_preference === 'challenging'}
                    onChange={(e) => handlePreferenceChange('difficulty_preference', e.target.value)}
                  />
                  چیلنجنگ
                </label>
              </div>
            </div>

            <div className="preference-section">
              <h2>دلچسپیاں</h2>
              <p>مثالوں اور مواد کو اپنی مرضی کے مطابق بنانے کے لیے موضوعات یا علاقے شامل کریں:</p>
              <form onSubmit={handleAddInterest} className="interests-form">
                <input
                  type="text"
                  name="interest"
                  placeholder="دلچسپی شامل کریں (مثلاً، روبوٹکس، اے آئی، پائتھون)"
                  className="input-field"
                />
                <button type="submit" className="button button--primary">شامل کریں</button>
              </form>

              <div className="interests-list">
                {preferences.interests.map((interest, index) => (
                  <span key={index} className="interest-tag">
                    {interest}
                    <button
                      onClick={() => handleRemoveInterest(interest)}
                      className="remove-interest"
                      aria-label={`${interest} کو ہٹائیں`}
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
                ڈیفالٹ پر ری سیٹ کریں
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
    <Layout title="سیکھنے کی ترجیحات" description="اپنی ذاتی سیکھنے کی ترجیحات کا نظم کریں">
      <BrowserOnly>
        {() => <PreferencesPageContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default PreferencesPage;
