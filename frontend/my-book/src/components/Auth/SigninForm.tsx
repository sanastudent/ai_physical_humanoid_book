import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getBackendUrl } from '@site/src/utils/config';
import './auth.css';

// Zod schema for signin validation (T059)
const signinSchema = z.object({
  email: z
    .string()
    .min(1, 'Email is required')
    .email('Please enter a valid email address'),
  password: z.string().min(1, 'Password is required'),
  rememberMe: z.boolean().optional(), // T060
});

type SigninFormData = z.infer<typeof signinSchema>;

interface SigninFormProps {
  onSigninSuccess: () => void; // T065: Callback to redirect to main content
}

const SigninForm: React.FC<SigninFormProps> = ({ onSigninSuccess }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [serverError, setServerError] = useState<string | null>(null);

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<SigninFormData>({
    resolver: zodResolver(signinSchema),
    defaultValues: {
      rememberMe: false,
    },
  });

  const onSubmit = async (data: SigninFormData) => {
    setIsLoading(true);
    setServerError(null);

    try {
      // Get backend URL from config utility
      const backendUrl = getBackendUrl();

      // T061: Integrate with POST /auth/signin endpoint
      const response = await fetch(`${backendUrl}/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Important: Include cookies for session
        body: JSON.stringify({
          email: data.email,
          password: data.password,
          remember_me: data.rememberMe || false, // T060: Remember me support
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();

        // T062: Handle invalid credentials error
        if (response.status === 401) {
          throw new Error('Invalid email or password. Please try again.');
        }

        throw new Error(errorData.detail || 'Failed to sign in');
      }

      // Successfully signed in, session cookie is set
      // T065: Redirect to main content after successful signin
      onSigninSuccess();

    } catch (error) {
      // T063: Network error handling with toast notifications
      if (error instanceof Error) {
        setServerError(error.message);
      } else {
        setServerError('An unexpected error occurred. Please try again.');
      }
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <div className="auth-form-card">
        <h2 className="auth-form-title">Welcome Back</h2>
        <p className="auth-form-subtitle">
          Sign in to access your personalized content
        </p>

        <form onSubmit={handleSubmit(onSubmit)} className="auth-form">
          {/* Email Field */}
          <div className="form-group">
            <label htmlFor="email" className="form-label">
              Email Address
            </label>
            <input
              id="email"
              type="email"
              className={`form-input ${errors.email ? 'form-input-error' : ''}`}
              placeholder="you@example.com"
              {...register('email')}
              disabled={isLoading}
              autoComplete="email"
            />
            {errors.email && (
              <p className="form-error">{errors.email.message}</p>
            )}
          </div>

          {/* Password Field */}
          <div className="form-group">
            <label htmlFor="password" className="form-label">
              Password
            </label>
            <input
              id="password"
              type="password"
              className={`form-input ${errors.password ? 'form-input-error' : ''}`}
              placeholder="Enter your password"
              {...register('password')}
              disabled={isLoading}
              autoComplete="current-password"
            />
            {errors.password && (
              <p className="form-error">{errors.password.message}</p>
            )}
          </div>

          {/* Remember Me Checkbox (T060) */}
          <div className="form-group">
            <label className="checkbox-label remember-me">
              <input
                type="checkbox"
                {...register('rememberMe')}
                disabled={isLoading}
              />
              <span>Remember me for 30 days</span>
            </label>
            <p className="form-hint">
              Keep me signed in on this device (extends session from 24 hours to 30 days)
            </p>
          </div>

          {/* Server Error Display (T062) */}
          {serverError && (
            <div className="form-error-banner">
              <svg className="form-error-icon" fill="currentColor" viewBox="0 0 20 20">
                <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
              </svg>
              {serverError}
            </div>
          )}

          {/* Submit Button */}
          <button
            type="submit"
            className="form-submit-button"
            disabled={isLoading}
          >
            {isLoading ? (
              <>
                <svg className="form-spinner" viewBox="0 0 24 24">
                  <circle className="form-spinner-circle" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" fill="none" />
                </svg>
                Signing In...
              </>
            ) : (
              'Sign In'
            )}
          </button>
        </form>

        <div className="auth-form-footer">
          <p>
            Don't have an account?{' '}
            <a href="/signup" className="auth-link">
              Sign up
            </a>
          </p>
          {/* Future: Add password reset link */}
          {/* <p>
            <a href="/forgot-password" className="auth-link">
              Forgot your password?
            </a>
          </p> */}
        </div>
      </div>
    </div>
  );
};

export default SigninForm;
