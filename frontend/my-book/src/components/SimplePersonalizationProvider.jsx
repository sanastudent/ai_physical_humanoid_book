import React, { createContext, useContext, useState, useEffect } from 'react';
import { safeLocalStorage } from './utils/safeLocalStorage';

// Check if running in browser environment
const isBrowser = typeof window !== 'undefined';

// Define the user preferences interface
const DEFAULT_PREFERENCES = {
  interests: [],
  experience_level: 'intermediate',
  preferred_examples: []
};

// Create the context
const SimplePersonalizationContext = createContext(undefined);

// SimplePersonalizationProvider component
export const SimplePersonalizationProvider = ({ children }) => {
  const [preferences, setPreferencesState] = useState(DEFAULT_PREFERENCES);
  const [isPersonalized, setIsPersonalizedState] = useState(false);
  const [isClient, setIsClient] = useState(false);

  // Initialize from localStorage on client side only
  useEffect(() => {
    if (isBrowser) {
      setIsClient(true);

      // Load preferences
      const savedPreferences = safeLocalStorage.get('simpleBookPreferences');
      if (savedPreferences) {
        try {
          setPreferencesState(JSON.parse(savedPreferences));
        } catch (error) {
          console.warn('Failed to parse saved preferences:', error);
          setPreferencesState(DEFAULT_PREFERENCES);
        }
      }

      // Load personalization status
      const savedPersonalized = safeLocalStorage.get('simpleBookIsPersonalized');
      if (savedPersonalized !== null) {
        try {
          setIsPersonalizedState(JSON.parse(savedPersonalized));
        } catch (error) {
          console.warn('Failed to parse personalization status:', error);
          setIsPersonalizedState(false);
        }
      }
    }
  }, []);

  // Save preferences to localStorage whenever they change (client side only)
  useEffect(() => {
    if (isClient) {
      safeLocalStorage.set('simpleBookPreferences', JSON.stringify(preferences));
    }
  }, [preferences, isClient]);

  // Save personalization status to localStorage whenever it changes (client side only)
  useEffect(() => {
    if (isClient) {
      safeLocalStorage.set('simpleBookIsPersonalized', JSON.stringify(isPersonalized));
    }
  }, [isPersonalized, isClient]);

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

  const setIsPersonalized = (value) => {
    setIsPersonalizedState(value);
  };

  const value = {
    preferences,
    setPreferences,
    addInterest,
    removeInterest,
    isPersonalized,
    setIsPersonalized
  };

  return (
    <SimplePersonalizationContext.Provider value={value}>
      {children}
    </SimplePersonalizationContext.Provider>
  );
};

// Custom hook to use the simple personalization context
export const useSimplePersonalization = () => {
  const context = useContext(SimplePersonalizationContext);
  if (context === undefined) {
    throw new Error('useSimplePersonalization must be used within a SimplePersonalizationProvider');
  }
  return context;
};