import React from "react";
import Layout from "@theme/Layout";
import SignupForm from "../components/auth/SignupForm";

export default function Signup() {
    return (
        <Layout title="Sign Up" description="Create your account">
            <SignupForm />
        </Layout>
    );
}
