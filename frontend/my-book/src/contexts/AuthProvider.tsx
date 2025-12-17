import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';
import { getCurrentUser, signOut as authSignOut } from '../utils/authClient';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getBackendUrl } from '@site/src/utils/config';

interface User {
  id: string;
  email: string;
  createdAt?: string;
  updatedAt?: string;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  loading: boolean;
  signIn: (email: string, password: string, rememberMe?: boolean) => Promise<boolean>;
  signOut: () => Promise<boolean>;
  signUp: (email: string, password: string, userData?: any) => Promise<boolean>;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Get backend URL from config utility
  const BACKEND_URL = getBackendUrl();

  const refreshSession = async () => {
    // Skip during SSR
    if (!ExecutionEnvironment.canUseDOM) {
      setLoading(false);
      return;
    }

    try {
      const currentUser = await getCurrentUser();
      setUser(currentUser);
    } catch (error) {
      console.error('Failed to refresh session:', error);
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    refreshSession();
  }, []);

  const signIn = async (email: string, password: string, rememberMe: boolean = false): Promise<boolean> => {
    try {
      const response = await fetch(`${BACKEND_URL}/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include session cookie
        body: JSON.stringify({ email, password, remember_me: rememberMe }),
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data);
        return true;
      }
      return false;
    } catch (error) {
      console.error('Sign in error:', error);
      return false;
    }
  };

  const signUp = async (email: string, password: string, userData?: any): Promise<boolean> => {
    try {
      const response = await fetch(`${BACKEND_URL}/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include session cookie
        body: JSON.stringify({ email, password, ...userData }),
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data);
        return true;
      }
      return false;
    } catch (error) {
      console.error('Sign up error:', error);
      return false;
    }
  };

  const signOut = async (): Promise<boolean> => {
    try {
      const success = await authSignOut();
      if (success) {
        setUser(null);
      }
      return success;
    } catch (error) {
      console.error('Sign out error:', error);
      return false;
    }
  };

  const value: AuthContextType = {
    user,
    isAuthenticated: !!user,
    loading,
    signIn,
    signOut,
    signUp,
    refreshSession
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};