export const safeLocalStorage = {
  get: (key) => {
    if (typeof window !== 'undefined' && window.localStorage) {
      try {
        return window.localStorage.getItem(key);
      } catch (error) {
        console.warn('Error reading from localStorage:', error);
        return null;
      }
    }
    return null;
  },

  set: (key, value) => {
    if (typeof window !== 'undefined' && window.localStorage) {
      try {
        window.localStorage.setItem(key, value);
      } catch (error) {
        console.warn('Error writing to localStorage:', error);
      }
    }
  },

  remove: (key) => {
    if (typeof window !== 'undefined' && window.localStorage) {
      try {
        window.localStorage.removeItem(key);
      } catch (error) {
        console.warn('Error removing from localStorage:', error);
      }
    }
  }
};