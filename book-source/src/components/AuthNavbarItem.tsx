import React from 'react';
import { useSession, signOut } from '../lib/authClient';
import Link from '@docusaurus/Link';

export default function AuthNavbarItem({ mobile }) {
    const { data: session } = useSession();

    if (session?.user) {
        return (
            <button
                onClick={() => signOut()}
                className="clean-btn navbar__item navbar__link"
                style={{ cursor: 'pointer', background: 'none', border: 'none', padding: 'var(--ifm-navbar-item-padding-vertical) var(--ifm-navbar-item-padding-horizontal)' }}
            >
                Sign Out
            </button>
        );
    }

    return (
        <Link
            to="/signin"
            className="navbar__item navbar__link"
        >
            Sign In
        </Link>
    );
}
