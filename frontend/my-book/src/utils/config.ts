/**
 * Configuration utility for accessing Docusaurus custom fields
 * SSR-safe way to access environment variables
 */

/**
 * Safely get environment variable value
 * Returns undefined if process is not available (browser environment)
 */
function getEnvVar(name: string): string | undefined {
  if (typeof process !== 'undefined' && process.env) {
    return process.env[name];
  }
  return undefined;
}

/**
 * Get the backend URL from the Docusaurus siteConfig
 * Falls back to localhost if not available (during SSR)
 */
export function getBackendUrl(): string {
  // During SSR or when window is not available
  if (typeof window === 'undefined') {
    return getEnvVar('REACT_APP_BACKEND_URL') || 'http://localhost:8000';
  }

  // Access the Docusaurus site config from window
  // @ts-ignore - Docusaurus injects this at runtime
  const siteConfig = window.docusaurus?.siteConfig;

  if (siteConfig?.customFields?.backendUrl) {
    return siteConfig.customFields.backendUrl as string;
  }

  // Fallback to default
  return getEnvVar('REACT_APP_BACKEND_URL') || 'http://localhost:8000';
}
