import React, { useState } from "react";
import { signUp } from "../../lib/authClient";
import styles from "./AuthForms.module.css";

interface SignupData {
    name: string;
    email: string;
    password: string;
    confirmPassword: string;
    softwareBackground: string;
    hardwareExperience: string;
    learningGoals: string;
    hasRoboticsExperience: boolean;
}

export default function SignupForm() {
    const [step, setStep] = useState(1);
    const [formData, setFormData] = useState<SignupData>({
        name: "",
        email: "",
        password: "",
        confirmPassword: "",
        softwareBackground: "beginner",
        hardwareExperience: "none",
        learningGoals: "",
        hasRoboticsExperience: false,
    });
    const [error, setError] = useState("");
    const [loading, setLoading] = useState(false);

    const handleInputChange = (
        e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>
    ) => {
        const { name, value, type } = e.target;
        setFormData((prev) => ({
            ...prev,
            [name]: type === "checkbox" ? (e.target as HTMLInputElement).checked : value,
        }));
    };

    const handleStep1Submit = (e: React.FormEvent) => {
        e.preventDefault();
        setError("");

        if (!formData.name || !formData.email || !formData.password) {
            setError("Please fill in all fields");
            return;
        }

        if (formData.password !== formData.confirmPassword) {
            setError("Passwords do not match");
            return;
        }

        if (formData.password.length < 8) {
            setError("Password must be at least 8 characters");
            return;
        }

        setStep(2);
    };

    const handleFinalSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setError("");
        setLoading(true);

        try {
            const result = await signUp.email({
                email: formData.email,
                password: formData.password,
                name: formData.name,
                softwareBackground: formData.softwareBackground,
                hardwareExperience: formData.hardwareExperience,
                learningGoals: formData.learningGoals,
                hasRoboticsExperience: formData.hasRoboticsExperience,
            });

            if (result.error) {
                setError(result.error.message || "Signup failed");
            } else {
                // Redirect to home page or dashboard
                window.location.href = "/";
            }
        } catch (err) {
            setError("An error occurred during signup");
            console.error(err);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className={styles.authContainer}>
            <div className={styles.authCard}>
                <h2 className={styles.authTitle}>Create Your Account</h2>
                <div className={styles.stepIndicator}>
                    <span className={step === 1 ? styles.activeStep : styles.completedStep}>
                        1. Basic Info
                    </span>
                    <span className={styles.stepDivider}>→</span>
                    <span className={step === 2 ? styles.activeStep : styles.inactiveStep}>
                        2. Background
                    </span>
                </div>

                {error && <div className={styles.error}>{error}</div>}

                {step === 1 && (
                    <form onSubmit={handleStep1Submit} className={styles.form}>
                        <div className={styles.formGroup}>
                            <label htmlFor="name">Full Name</label>
                            <input
                                type="text"
                                id="name"
                                name="name"
                                value={formData.name}
                                onChange={handleInputChange}
                                placeholder="John Doe"
                                required
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="email">Email Address</label>
                            <input
                                type="email"
                                id="email"
                                name="email"
                                value={formData.email}
                                onChange={handleInputChange}
                                placeholder="john@example.com"
                                required
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="password">Password</label>
                            <input
                                type="password"
                                id="password"
                                name="password"
                                value={formData.password}
                                onChange={handleInputChange}
                                placeholder="At least 8 characters"
                                required
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="confirmPassword">Confirm Password</label>
                            <input
                                type="password"
                                id="confirmPassword"
                                name="confirmPassword"
                                value={formData.confirmPassword}
                                onChange={handleInputChange}
                                placeholder="Re-enter your password"
                                required
                            />
                        </div>

                        <button type="submit" className={styles.submitButton}>
                            Next: Tell Us About Yourself →
                        </button>
                    </form>
                )}

                {step === 2 && (
                    <form onSubmit={handleFinalSubmit} className={styles.form}>
                        <p className={styles.stepDescription}>
                            Help us personalize your learning experience
                        </p>

                        <div className={styles.formGroup}>
                            <label htmlFor="softwareBackground">Software Background</label>
                            <select
                                id="softwareBackground"
                                name="softwareBackground"
                                value={formData.softwareBackground}
                                onChange={handleInputChange}
                            >
                                <option value="beginner">Beginner - Just starting out</option>
                                <option value="intermediate">Intermediate - Some experience</option>
                                <option value="advanced">Advanced - Experienced developer</option>
                            </select>
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="hardwareExperience">Hardware Experience</label>
                            <select
                                id="hardwareExperience"
                                name="hardwareExperience"
                                value={formData.hardwareExperience}
                                onChange={handleInputChange}
                            >
                                <option value="none">None - No hardware experience</option>
                                <option value="basic">Basic - Tinkered with electronics</option>
                                <option value="intermediate">Intermediate - Built some projects</option>
                                <option value="advanced">Advanced - Extensive experience</option>
                            </select>
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="learningGoals">What are your learning goals?</label>
                            <textarea
                                id="learningGoals"
                                name="learningGoals"
                                value={formData.learningGoals}
                                onChange={handleInputChange}
                                placeholder="Tell us what you want to learn or build..."
                                rows={4}
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label className={styles.checkboxLabel}>
                                <input
                                    type="checkbox"
                                    name="hasRoboticsExperience"
                                    checked={formData.hasRoboticsExperience}
                                    onChange={handleInputChange}
                                />
                                <span>I have prior experience with robotics</span>
                            </label>
                        </div>

                        <div className={styles.buttonGroup}>
                            <button
                                type="button"
                                onClick={() => setStep(1)}
                                className={styles.backButton}
                            >
                                ← Back
                            </button>
                            <button
                                type="submit"
                                className={styles.submitButton}
                                disabled={loading}
                            >
                                {loading ? "Creating Account..." : "Complete Signup"}
                            </button>
                        </div>
                    </form>
                )}

                <div className={styles.authFooter}>
                    Already have an account? <a href="/signin">Sign in</a>
                </div>
            </div>
        </div>
    );
}
