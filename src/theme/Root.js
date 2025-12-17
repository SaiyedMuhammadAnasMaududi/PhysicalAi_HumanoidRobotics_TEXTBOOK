import React, { useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Default implementation, that you can customize
export default function Root({children}) {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Inject backend configuration from Docusaurus customFields to window object
    // This makes it accessible to non-React JavaScript modules
    if (typeof window !== 'undefined' && siteConfig?.customFields) {
      window.docusaurus = window.docusaurus || {};
      window.docusaurus.customFields = siteConfig.customFields;

      // Log configuration for debugging (only in development)
      if (process.env.NODE_ENV === 'development') {
        console.log('Backend configuration injected:', {
          backendUrl: siteConfig.customFields.backendUrl,
          streamingEndpoint: siteConfig.customFields.streamingEndpoint,
          timeoutMs: siteConfig.customFields.timeoutMs,
        });
      }
    }
  }, [siteConfig]);

  return (
    <>
      {children}
    </>
  );
}
