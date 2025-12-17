/**
 * Animated typing indicator component showing three dots animation
 */

/**
 * Create TypingIndicator component
 * @param {Object} [options] - Options for the typing indicator
 * @param {string} [options.color] - Color of the typing dots
 * @param {number} [options.size=8] - Size of the typing dots in pixels
 * @param {boolean} [options.visible=true] - Whether the indicator is visible
 * @returns {HTMLElement} Typing indicator element
 */
export function TypingIndicator(options = {}) {
  const {
    color = 'var(--ifm-color-emphasis-600)',
    size = 8,
    visible = true
  } = options;

  const indicator = document.createElement('div');
  indicator.className = 'typing-indicator';
  indicator.style.display = visible ? 'flex' : 'none';
  indicator.setAttribute('role', 'status');
  indicator.setAttribute('aria-label', 'Assistant is typing');
  indicator.setAttribute('aria-live', 'polite');

  // Create the three dots
  for (let i = 0; i < 3; i++) {
    const dot = document.createElement('div');
    dot.className = 'typing-dot';
    dot.style.width = `${size}px`;
    dot.style.height = `${size}px`;
    dot.style.backgroundColor = color;
    dot.style.borderRadius = '50%';
    dot.style.margin = '0 2px';
    dot.style.animation = 'typing 1.4s infinite ease-in-out';
    dot.style.animationDelay = `${i * 0.2}s`;

    indicator.appendChild(dot);
  }

  // Add CSS for the animation if not already present
  addAnimationStyles();

  // Method to show the indicator
  indicator.show = () => {
    indicator.style.display = 'flex';
  };

  // Method to hide the indicator
  indicator.hide = () => {
    indicator.style.display = 'none';
  };

  // Method to set visibility
  indicator.setVisibility = (isVisible) => {
    indicator.style.display = isVisible ? 'flex' : 'none';
  };

  return indicator;
}

/**
 * Add the required CSS animation styles if they don't exist
 */
function addAnimationStyles() {
  // Check if styles are already added
  if (document.querySelector('#typing-indicator-styles')) {
    return;
  }

  const style = document.createElement('style');
  style.id = 'typing-indicator-styles';
  style.textContent = `
    @keyframes typing {
      0%, 60%, 100% { transform: translateY(0); }
      30% { transform: translateY(-5px); }
    }
  `;

  document.head.appendChild(style);
}

// Create a default typing indicator
export function createDefaultTypingIndicator() {
  return TypingIndicator();
}

// Export for use in other modules
export default TypingIndicator;