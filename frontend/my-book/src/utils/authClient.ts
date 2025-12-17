/**
 * Authentication Client Utility
 *
 * Provides helper functions for checking authentication status
 */

import { getBackendUrl } from './config';

export interface User {
  id: string;
  email: string;
  createdAt: string;
  updatedAt: string;
}

export interface SessionResponse {
  user: User | null;
  session: {
    active: boolean;
  } | null;
}

/**
 * Check if user is currently authenticated
 * @returns Promise<boolean> - true if authenticated, false otherwise
 */
export async function isAuthenticated(): Promise<boolean> {
  try {
    const backendUrl = getBackendUrl();

    const response = await fetch(`${backendUrl}/auth/session`, {
      method: 'GET',
      credentials: 'include', // Include session cookie
    });

    if (!response.ok) {
      return false;
    }

    const data: SessionResponse = await response.json();
    return data.user !== null;
  } catch (error) {
    console.error('Error checking authentication:', error);
    return false;
  }
}

/**
 * Get current user session
 * @returns Promise<User | null> - User object if authenticated, null otherwise
 */
export async function getCurrentUser(): Promise<User | null> {
  try {
    const backendUrl = getBackendUrl();

    const response = await fetch(`${backendUrl}/auth/session`, {
      method: 'GET',
      credentials: 'include', // Include session cookie
    });

    if (!response.ok) {
      return null;
    }

    const data: SessionResponse = await response.json();
    return data.user;
  } catch (error) {
    console.error('Error getting current user:', error);
    return null;
  }
}

/**
 * Sign out the current user
 * @returns Promise<boolean> - true if signout successful, false otherwise
 */
export async function signOut(): Promise<boolean> {
  try {
    const backendUrl = getBackendUrl();

    const response = await fetch(`${backendUrl}/auth/signout`, {
      method: 'POST',
      credentials: 'include', // Include session cookie
    });

    return response.ok;
  } catch (error) {
    console.error('Error signing out:', error);
    return false;
  }
}