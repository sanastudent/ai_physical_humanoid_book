import React, { useState } from 'react';
import { useForm, Controller } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getBackendUrl } from '@site/src/utils/config';
import './auth.css';

// Zod schema for background validation (T042)
const backgroundSchema = z.object({
  // Software background (T037)
  softwareExperienceLevel: z.enum(['beginner', 'intermediate', 'advanced'], {
    required_error: 'Please select your software development experience level',
  }),
  preferredLanguages: z
    .array(z.string())
    .min(1, 'Please select at least one programming language'),
  preferredFrameworks: z.array(z.string()).optional(),

  // Hardware background (T038)
  hardwareExperienceLevel: z.enum(['beginner', 'intermediate', 'advanced'], {
    required_error: 'Please select your hardware development experience level',
  }),
  preferredPlatforms: z
    .array(z.string())
    .min(1, 'Please select at least one development platform'),
  deviceTypes: z.array(z.string()).optional(),
});

type BackgroundFormData = z.infer<typeof backgroundSchema>;

interface BackgroundQuestionsFormProps {
  onComplete: () => void; // T047: Callback to redirect to main content
}

// Predefined options based on validators.py
const LANGUAGES = [
  'Python', 'JavaScript', 'TypeScript', 'Java', 'C++', 'C#', 'Go', 'Rust',
  'Ruby', 'PHP', 'Swift', 'Kotlin', 'R', 'MATLAB', 'SQL', 'Shell', 'Other'
];

const FRAMEWORKS = [
  'React', 'Vue', 'Angular', 'FastAPI', 'Django', 'Flask', 'Express', 'NestJS',
  'Spring', 'ASP.NET', 'Ruby on Rails', 'Laravel', 'Fiber', 'Gin', 'Other'
];

const PLATFORMS = [
  'desktop', 'mobile', 'web', 'embedded', 'cloud', 'iot', 'other'
];

const DEVICE_TYPES = [
  'laptop', 'desktop', 'smartphone', 'tablet', 'raspberry-pi', 'arduino',
  'microcontroller', 'fpga', 'gpu', 'server', 'other'
];

const BackgroundQuestionsForm: React.FC<BackgroundQuestionsFormProps> = ({ onComplete }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [serverError, setServerError] = useState<string | null>(null);

  const {
    register,
    handleSubmit,
    control,
    formState: { errors },
    watch,
  } = useForm<BackgroundFormData>({
    resolver: zodResolver(backgroundSchema),
    defaultValues: {
      preferredLanguages: [],
      preferredFrameworks: [],
      preferredPlatforms: [],
      deviceTypes: [],
    },
  });

  const selectedLanguages = watch('preferredLanguages') || [];
  const selectedFrameworks = watch('preferredFrameworks') || [];
  const selectedPlatforms = watch('preferredPlatforms') || [];
  const selectedDevices = watch('deviceTypes') || [];

  const onSubmit = async (data: BackgroundFormData) => {
    setIsLoading(true);
    setServerError(null);

    try {
      // Get backend URL from config utility
      const backendUrl = getBackendUrl();

      // T040: Integrate with POST /background endpoint
      const response = await fetch(`${backendUrl}/background`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include session cookie
        body: JSON.stringify({
          software: {
            experience_level: data.softwareExperienceLevel,
            preferred_languages: data.preferredLanguages,
            preferred_frameworks: data.preferredFrameworks || [],
          },
          hardware: {
            experience_level: data.hardwareExperienceLevel,
            preferred_platforms: data.preferredPlatforms,
            device_types: data.deviceTypes || [],
          },
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to save background information');
      }

      // Successfully saved background data
      // T047: Auto-redirect to main content after completing background questions
      onComplete();

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

  // Helper to toggle multi-select checkboxes
  const handleCheckboxChange = (
    field: 'preferredLanguages' | 'preferredFrameworks' | 'preferredPlatforms' | 'deviceTypes',
    value: string,
    currentValues: string[]
  ) => {
    return currentValues.includes(value)
      ? currentValues.filter(v => v !== value)
      : [...currentValues, value];
  };

  return (
    <div className="auth-form-container">
      <div className="auth-form-card background-form">
        <h2 className="auth-form-title">Tell Us About Your Background</h2>
        <p className="auth-form-subtitle">
          Step 2 of 2: Help us personalize your learning experience
        </p>

        <form onSubmit={handleSubmit(onSubmit)} className="auth-form">
          {/* Software Development Section */}
          <div className="form-section">
            <h3 className="form-section-title">Software Development</h3>

            {/* Software Experience Level (T037) */}
            <div className="form-group">
              <label className="form-label">Experience Level *</label>
              <div className="radio-group">
                <label className="radio-label">
                  <input
                    type="radio"
                    value="beginner"
                    {...register('softwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Beginner
                </label>
                <label className="radio-label">
                  <input
                    type="radio"
                    value="intermediate"
                    {...register('softwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Intermediate
                </label>
                <label className="radio-label">
                  <input
                    type="radio"
                    value="advanced"
                    {...register('softwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Advanced
                </label>
              </div>
              {errors.softwareExperienceLevel && (
                <p className="form-error">{errors.softwareExperienceLevel.message}</p>
              )}
            </div>

            {/* Preferred Languages (T037) */}
            <div className="form-group">
              <label className="form-label">
                Preferred Programming Languages * (Select at least 1)
              </label>
              <div className="checkbox-grid">
                <Controller
                  name="preferredLanguages"
                  control={control}
                  render={({ field }) => (
                    <>
                      {LANGUAGES.map((lang) => (
                        <label key={lang} className="checkbox-label">
                          <input
                            type="checkbox"
                            value={lang}
                            checked={selectedLanguages.includes(lang)}
                            onChange={(e) => {
                              const newValues = handleCheckboxChange(
                                'preferredLanguages',
                                lang,
                                selectedLanguages
                              );
                              field.onChange(newValues);
                            }}
                            disabled={isLoading}
                          />
                          {lang}
                        </label>
                      ))}
                    </>
                  )}
                />
              </div>
              {errors.preferredLanguages && (
                <p className="form-error">{errors.preferredLanguages.message}</p>
              )}
            </div>

            {/* Preferred Frameworks (T037) */}
            <div className="form-group">
              <label className="form-label">
                Preferred Frameworks (Optional)
              </label>
              <div className="checkbox-grid">
                <Controller
                  name="preferredFrameworks"
                  control={control}
                  render={({ field }) => (
                    <>
                      {FRAMEWORKS.map((framework) => (
                        <label key={framework} className="checkbox-label">
                          <input
                            type="checkbox"
                            value={framework}
                            checked={selectedFrameworks.includes(framework)}
                            onChange={(e) => {
                              const newValues = handleCheckboxChange(
                                'preferredFrameworks',
                                framework,
                                selectedFrameworks
                              );
                              field.onChange(newValues);
                            }}
                            disabled={isLoading}
                          />
                          {framework}
                        </label>
                      ))}
                    </>
                  )}
                />
              </div>
            </div>
          </div>

          {/* Hardware Development Section */}
          <div className="form-section">
            <h3 className="form-section-title">Hardware Development</h3>

            {/* Hardware Experience Level (T038) */}
            <div className="form-group">
              <label className="form-label">Experience Level *</label>
              <div className="radio-group">
                <label className="radio-label">
                  <input
                    type="radio"
                    value="beginner"
                    {...register('hardwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Beginner
                </label>
                <label className="radio-label">
                  <input
                    type="radio"
                    value="intermediate"
                    {...register('hardwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Intermediate
                </label>
                <label className="radio-label">
                  <input
                    type="radio"
                    value="advanced"
                    {...register('hardwareExperienceLevel')}
                    disabled={isLoading}
                  />
                  Advanced
                </label>
              </div>
              {errors.hardwareExperienceLevel && (
                <p className="form-error">{errors.hardwareExperienceLevel.message}</p>
              )}
            </div>

            {/* Preferred Platforms (T038) */}
            <div className="form-group">
              <label className="form-label">
                Preferred Development Platforms * (Select at least 1)
              </label>
              <div className="checkbox-grid">
                <Controller
                  name="preferredPlatforms"
                  control={control}
                  render={({ field }) => (
                    <>
                      {PLATFORMS.map((platform) => (
                        <label key={platform} className="checkbox-label">
                          <input
                            type="checkbox"
                            value={platform}
                            checked={selectedPlatforms.includes(platform)}
                            onChange={(e) => {
                              const newValues = handleCheckboxChange(
                                'preferredPlatforms',
                                platform,
                                selectedPlatforms
                              );
                              field.onChange(newValues);
                            }}
                            disabled={isLoading}
                          />
                          {platform.charAt(0).toUpperCase() + platform.slice(1)}
                        </label>
                      ))}
                    </>
                  )}
                />
              </div>
              {errors.preferredPlatforms && (
                <p className="form-error">{errors.preferredPlatforms.message}</p>
              )}
            </div>

            {/* Device Types (T038) */}
            <div className="form-group">
              <label className="form-label">
                Familiar Device Types (Optional)
              </label>
              <div className="checkbox-grid">
                <Controller
                  name="deviceTypes"
                  control={control}
                  render={({ field }) => (
                    <>
                      {DEVICE_TYPES.map((device) => (
                        <label key={device} className="checkbox-label">
                          <input
                            type="checkbox"
                            value={device}
                            checked={selectedDevices.includes(device)}
                            onChange={(e) => {
                              const newValues = handleCheckboxChange(
                                'deviceTypes',
                                device,
                                selectedDevices
                              );
                              field.onChange(newValues);
                            }}
                            disabled={isLoading}
                          />
                          {device.split('-').map(word =>
                            word.charAt(0).toUpperCase() + word.slice(1)
                          ).join(' ')}
                        </label>
                      ))}
                    </>
                  )}
                />
              </div>
            </div>
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
                Saving Your Preferences...
              </>
            ) : (
              'Complete Signup'
            )}
          </button>
        </form>

        <div className="auth-form-footer">
          <p className="form-hint">
            You can update your preferences anytime from your profile settings
          </p>
        </div>
      </div>
    </div>
  );
};

export default BackgroundQuestionsForm;
