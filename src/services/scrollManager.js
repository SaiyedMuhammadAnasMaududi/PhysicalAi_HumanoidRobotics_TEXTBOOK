/**
 * Auto-scroll behavior implementation service
 */

/**
 * Initialize scroll management for a chat container
 * @param {HTMLElement} container - The chat container element
 * @param {Function} onScrollChange - Callback when scroll position changes
 * @returns {Object} Scroll manager with control methods
 */
export function initializeScrollManager(container, onScrollChange) {
  if (!container) {
    throw new Error('Container element is required for scroll management');
  }

  // Track scroll state
  const state = {
    isAtBottom: true,
    lastScrollTop: 0,
    autoScrollEnabled: true,
    container
  };

  // Store the initial scroll position
  state.lastScrollTop = container.scrollTop;

  // Handle scroll events
  const handleScroll = () => {
    const currentScrollTop = container.scrollTop;
    const isAtBottom = isScrolledToBottom(container);

    // Update state based on scroll position
    state.isAtBottom = isAtBottom;
    state.lastScrollTop = currentScrollTop;

    // If user scrolled up, disable auto-scroll
    if (!isAtBottom) {
      state.autoScrollEnabled = false;
    }

    // Notify of scroll change
    if (onScrollChange) {
      onScrollChange({
        isAtBottom,
        position: currentScrollTop,
        autoScrollEnabled: state.autoScrollEnabled
      });
    }
  };

  // Attach scroll event listener
  container.addEventListener('scroll', handleScroll);

  // Return scroll manager object with methods
  return {
    /**
     * Scroll to the bottom of the container
     * @param {boolean} [smooth=true] - Whether to use smooth scrolling
     */
    scrollToBottom: (smooth = true) => {
      if (smooth) {
        container.scrollTo({
          top: container.scrollHeight,
          behavior: 'smooth'
        });
      } else {
        container.scrollTop = container.scrollHeight;
      }

      // Update state after scrolling
      state.isAtBottom = true;
      state.autoScrollEnabled = true;
      state.lastScrollTop = container.scrollTop;

      // Notify of scroll change
      if (onScrollChange) {
        onScrollChange({
          isAtBottom: true,
          position: container.scrollHeight,
          autoScrollEnabled: true
        });
      }
    },

    /**
     * Scroll to a specific position
     * @param {number} position - The scroll position to go to
     * @param {boolean} [smooth=true] - Whether to use smooth scrolling
     */
    scrollTo: (position, smooth = true) => {
      if (smooth) {
        container.scrollTo({
          top: position,
          behavior: 'smooth'
        });
      } else {
        container.scrollTop = position;
      }

      // Update state after scrolling
      const currentScrollTop = container.scrollTop;
      state.isAtBottom = isScrolledToBottom(container);
      state.autoScrollEnabled = state.isAtBottom;
      state.lastScrollTop = currentScrollTop;

      // Notify of scroll change
      if (onScrollChange) {
        onScrollChange({
          isAtBottom: state.isAtBottom,
          position: currentScrollTop,
          autoScrollEnabled: state.autoScrollEnabled
        });
      }
    },

    /**
     * Enable auto-scroll behavior
     */
    enableAutoScroll: () => {
      state.autoScrollEnabled = true;
      state.isAtBottom = isScrolledToBottom(container);

      // Notify of change
      if (onScrollChange) {
        onScrollChange({
          isAtBottom: state.isAtBottom,
          position: state.lastScrollTop,
          autoScrollEnabled: true
        });
      }
    },

    /**
     * Disable auto-scroll behavior
     */
    disableAutoScroll: () => {
      state.autoScrollEnabled = false;

      // Notify of change
      if (onScrollChange) {
        onScrollChange({
          isAtBottom: state.isAtBottom,
          position: state.lastScrollTop,
          autoScrollEnabled: false
        });
      }
    },

    /**
     * Check if auto-scroll is enabled
     * @returns {boolean} Whether auto-scroll is enabled
     */
    isAutoScrollEnabled: () => {
      return state.autoScrollEnabled;
    },

    /**
     * Check if user is at bottom of container
     * @returns {boolean} Whether user is at bottom
     */
    isAtBottom: () => {
      return state.isAtBottom;
    },

    /**
     * Get current scroll position
     * @returns {number} Current scroll position
     */
    getCurrentPosition: () => {
      return container.scrollTop;
    },

    /**
     * Check if should auto-scroll based on current state
     * @returns {boolean} Whether auto-scroll should occur
     */
    shouldAutoScroll: () => {
      return state.autoScrollEnabled && state.isAtBottom;
    },

    /**
     * Update scroll position after content changes
     * @param {boolean} [forceScrollToBottom=false] - Whether to force scroll to bottom
     */
    updateScrollPosition: (forceScrollToBottom = false) => {
      if (forceScrollToBottom || state.autoScrollEnabled) {
        // Small delay to ensure content has been rendered
        setTimeout(() => {
          if (forceScrollToBottom || state.autoScrollEnabled) {
            const isCurrentlyAtBottom = isScrolledToBottom(container);

            if (forceScrollToBottom || isCurrentlyAtBottom) {
              container.scrollTop = container.scrollHeight;
              state.isAtBottom = true;
              state.lastScrollTop = container.scrollHeight;
            }
          }
        }, 0);
      }
    },

    /**
     * Get scroll state information
     * @returns {Object} Scroll state
     */
    getState: () => {
      return {
        isAtBottom: state.isAtBottom,
        autoScrollEnabled: state.autoScrollEnabled,
        position: state.lastScrollTop
      };
    },

    /**
     * Clean up scroll manager (remove event listeners)
     */
    destroy: () => {
      container.removeEventListener('scroll', handleScroll);
    }
  };
}

/**
 * Check if the container is scrolled to the bottom
 * @param {HTMLElement} container - The container element to check
 * @returns {boolean} Whether the container is at the bottom
 */
function isScrolledToBottom(container) {
  // Calculate if we're within 5px of the bottom (tolerance for rounding)
  const tolerance = 5;
  const bottomPosition = container.scrollHeight - container.clientHeight;
  const currentPosition = container.scrollTop;

  return Math.abs(bottomPosition - currentPosition) <= tolerance;
}

/**
 * Calculate scroll percentage
 * @param {HTMLElement} container - The container element
 * @returns {number} Scroll percentage (0-100)
 */
export function getScrollPercentage(container) {
  if (!container || container.scrollHeight <= container.clientHeight) {
    return 100; // If content doesn't overflow, consider it 100% scrolled
  }

  const maxScroll = container.scrollHeight - container.clientHeight;
  const currentScroll = container.scrollTop;

  return Math.min(100, Math.max(0, (currentScroll / maxScroll) * 100));
}

/**
 * Check if container is scrollable
 * @param {HTMLElement} container - The container element
 * @returns {boolean} Whether the container has overflow content
 */
export function isScrollable(container) {
  return container.scrollHeight > container.clientHeight;
}

/**
 * Smooth scroll to element within container
 * @param {HTMLElement} container - The scroll container
 * @param {HTMLElement} targetElement - The element to scroll to
 * @param {string} [block='nearest'] - Scroll position ('start', 'center', 'end', 'nearest')
 * @param {string} [behavior='smooth'] - Scroll behavior ('auto', 'smooth')
 */
export function smoothScrollToElement(container, targetElement, block = 'nearest', behavior = 'smooth') {
  if (targetElement && container.contains(targetElement)) {
    targetElement.scrollIntoView({
      block,
      behavior,
      inline: 'nearest'
    });
  }
}