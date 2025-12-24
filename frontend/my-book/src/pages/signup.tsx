import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';
import { useHistory } from '@docusaurus/router';
import { translate } from '@docusaurus/Translate';

/**
 * Signup Page
 *
 * Simple signup with email and password
 */
const SignupPage: React.FC = () => {
  const history = useHistory();

  // Callback for successful signup
  const handleSignupSuccess = () => {
    // User is now authenticated (session cookie set)
    // Redirect to main content
    history.push('/');
  };

  return (
    <Layout
      title={translate({message: 'Sign Up'})}
      description={translate({message: 'Create your account'})}
    >
      <main>
        <SignupForm onSignupSuccess={handleSignupSuccess} />
      </main>
    </Layout>
  );
};

export default SignupPage;
