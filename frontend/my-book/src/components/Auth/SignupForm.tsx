import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getBackendUrl } from '@site/src/utils/config';
import './auth.css';

// Zod schema for signup validation (T035, T036)
const signupSchema = z.object({
  email: z
    .string()
    .min(1, 'Email is required')
    .email('Please enter a valid email address'),
  password: z
    .string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Password must contain at least one uppercase letter')
    .regex(/[a-z]/, 'Password must contain at least one lowercase letter')
    .regex(/[0-9]/, 'Password must contain at least one number'),
  confirmPassword: z.string().min(1, 'Please confirm your password'),
}).refine((data) => data.password === data.confirmPassword, {
  message: "Passwords don't match",
  path: ['confirmPassword'],
});

type SignupFormData = z.infer<typeof signupSchema>;

interface SignupFormProps {
  onSignupSuccess: () => void; // Callback to move to Step 2 (T046)
}

const SignupForm: React.FC<SignupFormProps> = ({ onSignupSuccess }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [serverError, setServerError] = useState<string | null>(null);

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<SignupFormData>({
    resolver: zodResolver(signupSchema),
    mode: 'onBlur', // Validate on blur for better UX
  });

  const onSubmit = async (data: SignupFormData) => {
    setIsLoading(true);
    setServerError(null);

    try {
      // Get backend URL from config utility
      const backendUrl = getBackendUrl();

      // T039: Integrate with POST /auth/signup endpoint
      const response = await fetch(`${backendUrl}/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Important: Include cookies for session
        body: JSON.stringify({
          email: data.email,
          password: data.password,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();

        // T043: Handle duplicate email error (status 400 or 409)
        if ((response.status === 400 || response.status === 409) &&
            (errorData.detail?.toLowerCase().includes('already') ||
             errorData.detail?.toLowerCase().includes('duplicate') ||
             errorData.detail?.toLowerCase().includes('exists'))) {
          throw new Error('This email is already registered. Please sign in instead.');
        }

        throw new Error(errorData.detail || 'Failed to create account');
      }

      // Successfully created account and session cookie is set
      // T046: Auto-login after successful signup (session cookie is automatically set)
      onSignupSuccess(); // Redirect to background questions (Step 2)

    } catch (error) {
      // T044: Network error handling
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
        <h2 className="auth-form-title">Create Your Account</h2>
        <p className="auth-form-subtitle">
          Step 1 of 2: Enter your email and password
        </p>

        <form onSubmit={handleSubmit(onSubmit)} className="auth-form">
          {/* Email Field with inline validation (T041) */}
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
            />
            {errors.email && (
              <p className="form-error">{errors.email.message}</p>
            )}
          </div>

          {/* Password Field with inline validation (T041) */}
          <div className="form-group">
            <label htmlFor="password" className="form-label">
              Password
            </label>
            <input
              id="password"
              type="password"
              className={`form-input ${errors.password ? 'form-input-error' : ''}`}
              placeholder="At least 8 characters"
              {...register('password')}
              disabled={isLoading}
            />
            {errors.password && (
              <p className="form-error">{errors.password.message}</p>
            )}
            <p className="form-hint">
              Must contain: 8+ characters, 1 uppercase, 1 lowercase, 1 number
            </p>
          </div>

          {/* Confirm Password Field */}
          <div className="form-group">
            <label htmlFor="confirmPassword" className="form-label">
              Confirm Password
            </label>
            <input
              id="confirmPassword"
              type="password"
              className={`form-input ${errors.confirmPassword ? 'form-input-error' : ''}`}
              placeholder="Re-enter your password"
              {...register('confirmPassword')}
              disabled={isLoading}
            />
            {errors.confirmPassword && (
              <p className="form-error">{errors.confirmPassword.message}</p>
            )}
          </div>

          {/* Server Error Display */}
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
                Creating Account...
              </>
            ) : (
              'Continue to Background Questions'
            )}
          </button>
        </form>

        <div className="auth-form-footer">
          <p>
            Already have an account?{' '}
            <a href="/signin" className="auth-link">
              Sign in
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SignupForm;
