// Mock auth client using localStorage for Docusaurus demo
// Note: In production, this should use a proper backend auth service

import React from 'react';

interface User {
    id: string;
    name: string;
    email: string;
    softwareBackground?: string;
    hardwareExperience?: string;
    learningGoals?: string;
    hasRoboticsExperience?: boolean;
}

interface Session {
    user: User | null;
}

// Mock session management
const getStoredUser = (): User | null => {
    if (typeof window === 'undefined') return null;
    const userStr = localStorage.getItem('mock_user');
    return userStr ? JSON.parse(userStr) : null;
};

const setStoredUser = (user: User | null) => {
    if (typeof window === 'undefined') return;
    if (user) {
        localStorage.setItem('mock_user', JSON.stringify(user));
    } else {
        localStorage.removeItem('mock_user');
    }
};

// Mock auth client
export const authClient = {
    useSession: () => {
        const [user, setUser] = React.useState<User | null>(getStoredUser);

        React.useEffect(() => {
            setUser(getStoredUser());
        }, []);

        return {
            data: { user } as Session,
            isPending: false,
        };
    },
};

// Mock signup
export const signUp = {
    email: async (data: any) => {
        const user: User = {
            id: Math.random().toString(36).substring(7),
            name: data.name,
            email: data.email,
            softwareBackground: data.softwareBackground,
            hardwareExperience: data.hardwareExperience,
            learningGoals: data.learningGoals,
            hasRoboticsExperience: data.hasRoboticsExperience,
        };

        setStoredUser(user);
        return { user, error: null };
    },
};

// Mock signin
export const signIn = {
    email: async (data: { email: string; password: string }) => {
        // For demo, just check if email exists in localStorage
        const existingUser = getStoredUser();

        if (existingUser && existingUser.email === data.email) {
            return { user: existingUser, error: null };
        }

        // Create new user for demo
        const user: User = {
            id: Math.random().toString(36).substring(7),
            name: data.email.split('@')[0],
            email: data.email,
        };

        setStoredUser(user);
        return { user, error: null };
    },
};

// Mock signout
export const signOut = async () => {
    setStoredUser(null);
    window.location.reload();
};

export const useSession = authClient.useSession;
