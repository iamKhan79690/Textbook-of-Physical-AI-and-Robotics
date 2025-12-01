import React from 'react';
import DocItemContent from '@theme-original/DocItem/Content';
import TranslateButton from '@site/src/components/translation/TranslateButton';
import PersonalizeButton from '@site/src/components/personalization/PersonalizeButton';

export default function DocItemContentWrapper(props) {
    return (
        <>
            <div style={{
                display: 'flex',
                flexWrap: 'wrap',
                gap: '12px',
                marginBottom: '24px',
                alignItems: 'center'
            }}>
                <TranslateButton />
                <PersonalizeButton />
            </div>
            <DocItemContent {...props} />
        </>
    );
}
