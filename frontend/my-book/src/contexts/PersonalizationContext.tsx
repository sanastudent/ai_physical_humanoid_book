import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';
import { safeLocalStorage } from '../utils/safeLocalStorage';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from './AuthProvider'; // Import the new AuthProvider
import { getBackendUrl } from '@site/src/utils/config';

interface UserPreferences {
  learning_style: 'visual' | 'auditory' | 'reading/writing' | 'kinesthetic' | 'multimodal';
  experience_level: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  interests: string[];
  difficulty_preference: string;
  preferred_examples: string[];
  accessibility_needs: string[];
}

interface User {
  id: string;
  email: string;
  createdAt?: string;
  updatedAt?: string;
}

interface UserBackground {
  userId: string;
  software: {
    id?: string;
    experience_level: string;
    preferred_languages: string[];
    preferred_frameworks: string[];
    createdAt?: string;
    updatedAt?: string;
  };
  hardware: {
    id?: string;
    experience_level: string;
    preferred_platforms: string[];
    device_types: string[];
    createdAt?: string;
    updatedAt?: string;
  };
}

interface PersonalizationContextType {
  preferences: UserPreferences;
  setPreferences: (prefs: UserPreferences) => void;
  addInterest: (interest: string) => void;
  removeInterest: (interest: string) => void;
  isPersonalized: (chapterId: string) => boolean;
  getPersonalizedContent: (chapterId: string) => string | null;
  setPersonalizedContent: (chapterId: string, content: string) => void;
  // Authentication state (now coming from AuthProvider)
  user: User | null;
  userBackground: UserBackground | null;
  isAuthenticated: boolean;
  loading: boolean;
  fetchUserBackground: () => Promise<void>;
}

const DEFAULT_PREFERENCES: UserPreferences = {
  learning_style: 'multimodal',
  experience_level: 'intermediate',
  interests: [],
  difficulty_preference: 'moderate',
  preferred_examples: [],
  accessibility_needs: []
};

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

export const PersonalizationProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [preferences, setPreferencesState] = useState<UserPreferences>(DEFAULT_PREFERENCES);
  const [personalizedContents, setPersonalizedContents] = useState<Record<string, string>>({});
  const [isClient, setIsClient] = useState(false);
  const [userBackground, setUserBackground] = useState<UserBackground | null>(null);

  // Get authentication state from AuthProvider
  const { user, isAuthenticated, loading: authLoading } = useAuth();

  // Get backend URL from config utility
  const BACKEND_URL = getBackendUrl();

  // Initialize from localStorage on client side only
  useEffect(() => {
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
  }, []);

  // Fetch user background when user logs in
  useEffect(() => {
    if (isAuthenticated && user && !userBackground) {
      fetchUserBackground();
    }
  }, [isAuthenticated, user]);

  // Fetch user background from backend (browser-only)
  const fetchUserBackground = async () => {
    // Skip during SSR
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    if (!user) {
      console.warn('No user logged in, cannot fetch background');
      return;
    }

    try {
      const response = await fetch(`${BACKEND_URL}/background`, {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json'
        }
      });

      if (response.ok) {
        const data = await response.json();
        setUserBackground(data);

        // Update preferences based on background data
        if (data.software) {
          setPreferencesState(prev => ({
            ...prev,
            experience_level: data.software.experience_level as any,
            preferred_examples: data.software.preferred_languages || []
          }));
        }
      } else if (response.status === 404) {
        console.info('User has not completed background questions yet');
        setUserBackground(null);
      } else {
        console.error('Failed to fetch background:', response.statusText);
      }
    } catch (error) {
      console.error('Error fetching user background:', error);
    }
  };

  // Save preferences to localStorage whenever they change (client side only)
  useEffect(() => {
    if (isClient) {
      safeLocalStorage.set('bookPreferences', JSON.stringify(preferences));
    }
  }, [preferences, isClient]);

  const setPreferences = (prefs: UserPreferences) => {
    setPreferencesState(prefs);
  };

  const addInterest = (interest: string) => {
    if (!preferences.interests.includes(interest)) {
      setPreferences({
        ...preferences,
        interests: [...preferences.interests, interest]
      });
    }
  };

  const removeInterest = (interest: string) => {
    setPreferences({
      ...preferences,
      interests: preferences.interests.filter(i => i !== interest)
    });
  };

  const isPersonalized = (chapterId: string): boolean => {
    return chapterId in personalizedContents;
  };

  const getPersonalizedContent = (chapterId: string): string | null => {
    return personalizedContents[chapterId] || null;
  };

  const setPersonalizedContent = (chapterId: string, content: string) => {
    setPersonalizedContents(prev => ({
      ...prev,
      [chapterId]: content
    }));

    // Also save to localStorage on client side
    if (isClient) {
      const savedPersonalized = safeLocalStorage.get('bookPersonalizedContents');
      let personalizedMap: Record<string, string> = {};
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

  const value: PersonalizationContextType = {
    preferences,
    setPreferences,
    addInterest,
    removeInterest,
    isPersonalized,
    getPersonalizedContent,
    setPersonalizedContent,
    // Authentication (now coming from AuthProvider)
    user,
    userBackground,
    isAuthenticated,
    loading: authLoading,
    fetchUserBackground
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = (): PersonalizationContextType => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};