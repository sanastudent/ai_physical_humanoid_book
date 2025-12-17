import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { getCurrentUser, signOut as authSignOut } from '@site/src/utils/authClient';
import './authNavbar.css';

/**
 * Authentication Navbar Item
 *
 * T066: Implement signout functionality
 * T067: Add signout button to navigation bar
 *
 * Shows "Sign In / Sign Up" for anonymous users
 * Shows user email + "Sign Out" button for authenticated users
 */
// Browser-only implementation of the AuthNavbarItem
const AuthNavbarItemContent: React.FC = () => {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userEmail, setUserEmail] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [showDropdown, setShowDropdown] = useState(false);
  const history = useHistory();

  // Check authentication status on mount
  useEffect(() => {
    checkAuthStatus();
  }, []);

  const checkAuthStatus = async () => {
    try {
      const user = await getCurrentUser();
      if (user) {
        setIsAuthenticated(true);
        setUserEmail(user.email);
      } else {
        setIsAuthenticated(false);
        setUserEmail(null);
      }
    } catch (error) {
      // Not authenticated or network error
      setIsAuthenticated(false);
      setUserEmail(null);
    } finally {
      setIsLoading(false);
    }
  };

  // T066: Signout functionality
  const handleSignout = async () => {
    try {
      const success = await authSignOut();

      if (success) {
        // Clear local state
        setIsAuthenticated(false);
        setUserEmail(null);
        setShowDropdown(false);

        // Redirect to home page
        history.push('/');
      } else {
        console.error('Failed to sign out');
      }
    } catch (error) {
      console.error('Signout error:', error);
    }
  };

  if (isLoading) {
    return null; // Don't show anything while loading
  }

  if (!isAuthenticated) {
    // Anonymous user: Show Sign In / Sign Up links
    return (
      <div className="auth-navbar-item">
        <a href="/signin" className="auth-navbar-link">
          Sign In
        </a>
        <span className="auth-navbar-separator">|</span>
        <a href="/signup" className="auth-navbar-link auth-navbar-signup">
          Sign Up
        </a>
      </div>
    );
  }

  // Authenticated user: Show email + dropdown with Sign Out
  return (
    <div className="auth-navbar-item">
      <button
        className="auth-navbar-user-button"
        onClick={() => setShowDropdown(!showDropdown)}
        onBlur={() => setTimeout(() => setShowDropdown(false), 200)}
      >
        <svg className="auth-navbar-user-icon" fill="currentColor" viewBox="0 0 20 20">
          <path fillRule="evenodd" d="M10 9a3 3 0 100-6 3 3 0 000 6zm-7 9a7 7 0 1114 0H3z" clipRule="evenodd" />
        </svg>
        <span className="auth-navbar-user-email">{userEmail}</span>
        <svg className="auth-navbar-dropdown-icon" fill="currentColor" viewBox="0 0 20 20">
          <path fillRule="evenodd" d="M5.293 7.293a1 1 0 011.414 0L10 10.586l3.293-3.293a1 1 0 111.414 1.414l-4 4a1 1 0 01-1.414 0l-4-4a1 1 0 010-1.414z" clipRule="evenodd" />
        </svg>
      </button>

      {showDropdown && (
        <div className="auth-navbar-dropdown">
          <a href="/preferences" className="auth-navbar-dropdown-item">
            <svg className="auth-navbar-dropdown-icon-item" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M11.49 3.17c-.38-1.56-2.6-1.56-2.98 0a1.532 1.532 0 01-2.286.948c-1.372-.836-2.942.734-2.106 2.106.54.886.061 2.042-.947 2.287-1.561.379-1.561 2.6 0 2.978a1.532 1.532 0 01.947 2.287c-.836 1.372.734 2.942 2.106 2.106a1.532 1.532 0 012.287.947c.379 1.561 2.6 1.561 2.978 0a1.533 1.533 0 012.287-.947c1.372.836 2.942-.734 2.106-2.106a1.533 1.533 0 01.947-2.287c1.561-.379 1.561-2.6 0-2.978a1.532 1.532 0 01-.947-2.287c.836-1.372-.734-2.942-2.106-2.106a1.532 1.532 0 01-2.287-.947zM10 13a3 3 0 100-6 3 3 0 000 6z" clipRule="evenodd" />
            </svg>
            Preferences
          </a>
          <button
            className="auth-navbar-dropdown-item"
            onClick={() => {
              // Scroll to the first personalization button on the page
              const personalizationBtn = document.querySelector('.personalize-btn');
              if (personalizationBtn) {
                personalizationBtn.scrollIntoView({ behavior: 'smooth' });
                // Focus on the button if possible
                (personalizationBtn as HTMLElement).focus();
              } else {
                // If no personalization button found, navigate to the first doc
                window.location.href = '/docs/intro';
              }
            }}
          >
            <svg className="auth-navbar-dropdown-icon-item" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M12.316 3.051a1 1 0 01.633 1.265l-4 12a1 1 0 11-1.898-.632l4-12a1 1 0 011.265-.633zM5.707 6.293a1 1 0 010 1.414L3.414 10l2.293 2.293a1 1 0 11-1.414 1.414l-3-3a1 1 0 010-1.414l3-3a1 1 0 011.414 0zm8.586 0a1 1 0 011.414 0l3 3a1 1 0 010 1.414l-3 3a1 1 0 11-1.414-1.414L16.586 10l-2.293-2.293a1 1 0 010-1.414z" clipRule="evenodd" />
            </svg>
            Personalize Content
          </button>
          <button
            className="auth-navbar-dropdown-item auth-navbar-signout"
            onClick={handleSignout}
          >
            <svg className="auth-navbar-dropdown-icon-item" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M3 3a1 1 0 00-1 1v12a1 1 0 102 0V4a1 1 0 00-1-1zm10.293 9.293a1 1 0 001.414 1.414l3-3a1 1 0 000-1.414l-3-3a1 1 0 10-1.414 1.414L14.586 9H7a1 1 0 100 2h7.586l-1.293 1.293z" clipRule="evenodd" />
            </svg>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
};

const AuthNavbarItem: React.FC = () => {
  return <BrowserOnly>{() => <AuthNavbarItemContent />}</BrowserOnly>;
};

export default AuthNavbarItem;
