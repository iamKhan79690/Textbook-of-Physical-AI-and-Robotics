import React, { useState } from "react";
import { useSession } from "../../lib/authClient";
import styles from "./TranslateButton.module.css";

interface TranslateButtonProps {
    pageContent?: string;
    pageTitle?: string;
}

export default function TranslateButton({ pageContent, pageTitle }: TranslateButtonProps) {
    const { data: session, isPending } = useSession();
    const [isTranslating, setIsTranslating] = useState(false);
    const [translatedContent, setTranslatedContent] = useState<string | null>(null);
    const [showTranslation, setShowTranslation] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // Don't show button if user is not authenticated
    if (isPending) {
        return null; // Loading session
    }

    if (!session?.user) {
        return (
            <div className={styles.notAuthMessage}>
                <p>🔒 Please <a href="/signin">sign in</a> to access Urdu translation</p>
            </div>
        );
    }

    const handleTranslate = async () => {
        if (translatedContent && showTranslation) {
            // Toggle off
            setShowTranslation(false);
            return;
        }

        if (translatedContent) {
            // Already translated, just show it
            setShowTranslation(true);
            return;
        }

        setIsTranslating(true);
        setError(null);

        try {
            // Get the current page content from DOM - try multiple selectors
            let contentToTranslate = "";

            // Try different selectors for Docusaurus content
            const selectors = [
                'article.theme-doc-markdown',
                'article',
                '.markdown',
                'main article',
                '[class*="docItemContainer"]'
            ];

            for (const selector of selectors) {
                const element = document.querySelector(selector);
                if (element?.textContent?.trim()) {
                    contentToTranslate = element.textContent.trim();
                    break;
                }
            }

            // Fallback to page content prop
            if (!contentToTranslate && pageContent) {
                contentToTranslate = pageContent;
            }

            if (!contentToTranslate || contentToTranslate.length < 10) {
                throw new Error("No content found to translate. Please make sure you're on a documentation page.");
            }

            // Call RAG server for translation
            const response = await fetch("http://localhost:8000/translate", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify({
                    text: contentToTranslate.substring(0, 5000), // Limit to 5000 chars for now
                    target_language: "urdu",
                    title: pageTitle || document.title,
                }),
            });

            if (!response.ok) {
                throw new Error("Translation failed");
            }

            const data = await response.json();
            setTranslatedContent(data.translated_text);
            setShowTranslation(true);
        } catch (err) {
            setError(err instanceof Error ? err.message : "Translation failed");
            console.error("Translation error:", err);
        } finally {
            setIsTranslating(false);
        }
    };

    return (
        <div className={styles.translateContainer}>
            <button
                onClick={handleTranslate}
                disabled={isTranslating}
                className={`${styles.translateButton} ${showTranslation ? styles.active : ""}`}
            >
                {isTranslating ? (
                    <>
                        <span className={styles.spinner}>⏳</span> Translating to Urdu...
                    </>
                ) : showTranslation ? (
                    <>📖 Show Original (English)</>
                ) : (
                    <>🌐 Translate to Urdu (اردو)</>
                )}
            </button>

            {error && (
                <div className={styles.error}>
                    <p>❌ {error}</p>
                    <button onClick={() => setError(null)} className={styles.dismissButton}>
                        Dismiss
                    </button>
                </div>
            )}

            {showTranslation && translatedContent && (
                <div className={styles.translatedContent}>
                    <div className={styles.translationHeader}>
                        <h3>📝 Urdu Translation (اردو ترجمہ)</h3>
                        <p className={styles.translationNote}>
                            AI-generated translation. For accuracy, refer to the original English text.
                        </p>
                    </div>
                    <div className={styles.translationText} dir="rtl" lang="ur">
                        {translatedContent}
                    </div>
                </div>
            )}
        </div>
    );
}
