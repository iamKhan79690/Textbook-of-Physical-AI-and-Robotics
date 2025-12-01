import React from "react";
import Layout from "@theme/Layout";
import SigninForm from "../components/auth/SigninForm";

export default function Signin() {
    return (
        <Layout title="Sign In" description="Sign in to your account">
            <SigninForm />
        </Layout>
    );
}
