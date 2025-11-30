import React, { useState } from "react";
import { useSession } from "../../lib/authClient";
import styles from "./PersonalizeButton.module.css";

interface PersonalizeButtonProps {
    pageContent?: string;
    pageTitle?: string;
}

export default function PersonalizeButton({ pageContent, pageTitle }: PersonalizeButtonProps) {
    const { data: session, isPending } = useSession();
    const [isPersonalizing, setIsPersonalizing] = useState(false);
    const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
    const [showPersonalized, setShowPersonalized] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // Don't show button if user is not authenticated
    if (isPending) {
        return null;
    }

    if (!session?.user) {
        return null; // Don't show anything if not logged in (Translate button handles the "please login" message)
    }

    const handlePersonalize = async () => {
        if (personalizedContent && showPersonalized) {
            setShowPersonalized(false);
            return;
        }

        if (personalizedContent) {
            setShowPersonalized(true);
            return;
        }

        setIsPersonalizing(true);
        setError(null);

        try {
            // Get content (reuse logic from TranslateButton or similar)
            let contentToPersonalize = "";
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
                    contentToPersonalize = element.textContent.trim();
                    break;
                }
            }

            if (!contentToPersonalize && pageContent) {
                contentToPersonalize = pageContent;
            }

            if (!contentToPersonalize || contentToPersonalize.length < 10) {
                throw new Error("No content found to personalize.");
            }

            // Call RAG server
            const response = await fetch("http://localhost:8000/personalize", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify({
                    text: contentToPersonalize.substring(0, 5000),
                    user_profile: {
                        softwareBackground: session.user.softwareBackground,
                        hardwareExperience: session.user.hardwareExperience,
                        learningGoals: session.user.learningGoals,
                        hasRoboticsExperience: session.user.hasRoboticsExperience
                    },
                    title: pageTitle || document.title,
                }),
            });

            if (!response.ok) {
                throw new Error("Personalization failed");
            }

            const data = await response.json();
            setPersonalizedContent(data.personalized_text);
            setShowPersonalized(true);
        } catch (err) {
            setError(err instanceof Error ? err.message : "Personalization failed");
            console.error("Personalization error:", err);
        } finally {
            setIsPersonalizing(false);
        }
    };

    return (
        <div className={styles.personalizeContainer}>
            <button
                onClick={handlePersonalize}
                disabled={isPersonalizing}
                className={`${styles.personalizeButton} ${showPersonalized ? styles.active : ""}`}
            >
                {isPersonalizing ? (
                    <>
                        <span className={styles.spinner}>✨</span> Personalizing...
                    </>
                ) : showPersonalized ? (
                    <>↩️ Show Original</>
                ) : (
                    <>✨ Personalize for Me</>
                )}
            </button>

            {error && (
                <div className={styles.error}>
                    <p>❌ {error}</p>
                    <button onClick={() => setError(null)} className={styles.dismissButton}>Dismiss</button>
                </div>
            )}

            {showPersonalized && personalizedContent && (
                <div className={styles.personalizedContent}>
                    <div className={styles.header}>
                        <h3>✨ Personalized Content</h3>
                        <p className={styles.note}>
                            Adapted for your profile:
                            {session.user.softwareBackground && ` ${session.user.softwareBackground} Software`}
                            {session.user.hardwareExperience && ` • ${session.user.hardwareExperience} Hardware`}
                        </p>
                    </div>
                    <div className={styles.content}>
                        {personalizedContent.split('\n').map((line, i) => (
                            <p key={i}>{line}</p>
                        ))}
                    </div>
                </div>
            )}
        </div>
    );
}
