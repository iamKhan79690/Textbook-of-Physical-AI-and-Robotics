import React, { useEffect } from 'react';
import ChatWidget from '../components/ChatWidget';

// Root component wrapper for Docusaurus
// This adds the ChatWidget to all pages
export default function Root({ children }) {
    useEffect(() => {
        console.log('✅ ChatWidget Root wrapper loaded');
    }, []);

    return (
        <>
            {children}
            <ChatWidget />
        </>
    );
}
