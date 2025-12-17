import React, { useState } from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';
import BackgroundQuestionsForm from '@site/src/components/Auth/BackgroundQuestionsForm';
import { useHistory } from '@docusaurus/router';

/**
 * Signup Page with Multi-step Flow
 *
 * T034: Multi-step form state management (Step 1 → Step 2)
 * T045: Signup page route
 * T046: Auto-login after successful signup
 * T047: Auto-redirect to main content after completing background questions
 */
const SignupPage: React.FC = () => {
  const [currentStep, setCurrentStep] = useState<1 | 2>(1);
  const history = useHistory();

  // T046: Callback for successful signup (auto-login via session cookie)
  const handleSignupSuccess = () => {
    // User is now authenticated (session cookie set)
    // Move to Step 2: Background Questions
    setCurrentStep(2);
  };

  // T047: Callback for completing background questions
  const handleBackgroundComplete = () => {
    // Redirect to main content (e.g., home page or documentation)
    history.push('/');
  };

  return (
    <Layout
      title="Sign Up"
      description="Create your account and personalize your learning experience"
    >
      <main>
        {/* Step 1: Email/Password (T032) */}
        {currentStep === 1 && (
          <SignupForm onSignupSuccess={handleSignupSuccess} />
        )}

        {/* Step 2: Background Questions (T033) */}
        {currentStep === 2 && (
          <BackgroundQuestionsForm onComplete={handleBackgroundComplete} />
        )}
      </main>
    </Layout>
  );
};

export default SignupPage;
