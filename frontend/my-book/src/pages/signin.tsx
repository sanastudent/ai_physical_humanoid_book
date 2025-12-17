import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '@site/src/components/Auth/SigninForm';
import { useHistory } from '@docusaurus/router';

/**
 * Signin Page
 *
 * T064: Signin page route
 * T065: Redirect to main content after successful signin
 */
const SigninPage: React.FC = () => {
  const history = useHistory();

  // T065: Callback for successful signin
  const handleSigninSuccess = () => {
    // User is now authenticated (session cookie set)
    // Redirect to main content (e.g., home page or documentation)
    // Check if there's a redirect URL in query params
    const urlParams = new URLSearchParams(window.location.search);
    const redirectTo = urlParams.get('redirect') || '/';

    history.push(redirectTo);
  };

  return (
    <Layout
      title="Sign In"
      description="Sign in to access your personalized learning experience"
    >
      <main>
        <SigninForm onSigninSuccess={handleSigninSuccess} />
      </main>
    </Layout>
  );
};

export default SigninPage;
