import React, { createContext, useContext, useState, useEffect } from 'react';
import { safeLocalStorage } from './utils/safeLocalStorage';

// Check if running in browser environment
const isBrowser = typeof window !== 'undefined';

// Define the default preferences
const DEFAULT_PREFERENCES = {
  learning_style: 'multimodal',
  experience_level: 'intermediate',
  interests: [],
  difficulty_preference: 'moderate',
  preferred_examples: [],
  accessibility_needs: []
};

// Create the context
const PersonalizationContext = createContext(undefined);

// PersonalizationProvider component
export const PersonalizationProvider = ({ children }) => {
  const [preferences, setPreferencesState] = useState(DEFAULT_PREFERENCES);
  const [personalizedContents, setPersonalizedContents] = useState({});
  const [isClient, setIsClient] = useState(false);

  // Initialize from localStorage on client side only
  useEffect(() => {
    if (isBrowser) {
      setIsClient(true);

      // Load preferences
      const savedPreferences = safeLocalStorage.get('bookPreferences');
      if (savedPreferences) {
        try {
          setPreferencesState(JSON.parse(savedPreferences));
        } catch (error) {
          console.warn('Failed to parse saved preferences:', error);
          setPreferencesState(DEFAULT_PREFERENCES);
        }
      }

      // Load personalized contents
      const savedPersonalized = safeLocalStorage.get('bookPersonalizedContents');
      if (savedPersonalized) {
        try {
          setPersonalizedContents(JSON.parse(savedPersonalized));
        } catch (error) {
          console.warn('Failed to parse saved personalized contents:', error);
          setPersonalizedContents({});
        }
      }
    }
  }, []);

  // Save preferences to localStorage whenever they change (client side only)
  useEffect(() => {
    if (isClient) {
      safeLocalStorage.set('bookPreferences', JSON.stringify(preferences));
    }
  }, [preferences, isClient]);

  const setPreferences = (prefs) => {
    setPreferencesState(prefs);
  };

  const addInterest = (interest) => {
    if (!preferences.interests.includes(interest)) {
      setPreferences({
        ...preferences,
        interests: [...preferences.interests, interest]
      });
    }
  };

  const removeInterest = (interest) => {
    setPreferences({
      ...preferences,
      interests: preferences.interests.filter(i => i !== interest)
    });
  };

  const isPersonalized = (chapterId) => {
    return chapterId in personalizedContents;
  };

  const getPersonalizedContent = (chapterId) => {
    return personalizedContents[chapterId] || null;
  };

  const setPersonalizedContent = (chapterId, content) => {
    setPersonalizedContents(prev => ({
      ...prev,
      [chapterId]: content
    }));

    // Also save to localStorage on client side
    if (isClient) {
      const savedPersonalized = safeLocalStorage.get('bookPersonalizedContents');
      let personalizedMap = {};
      if (savedPersonalized) {
        try {
          personalizedMap = JSON.parse(savedPersonalized);
        } catch (error) {
          console.warn('Failed to parse saved personalized contents:', error);
        }
      }
      personalizedMap[chapterId] = content;
      safeLocalStorage.set('bookPersonalizedContents', JSON.stringify(personalizedMap));
    }
  };

  const value = {
    preferences,
    setPreferences,
    addInterest,
    removeInterest,
    isPersonalized,
    getPersonalizedContent,
    setPersonalizedContent
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

// Custom hook to use the personalization context
export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};